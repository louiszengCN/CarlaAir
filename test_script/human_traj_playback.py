      
import carla
import json
import sys
import time
import os
import numpy as np
import cv2
import pygame

class TrajectoryPlayback:
    def __init__(self, trajectory_file, record=False, record_name="output.mp4", visual_style="walker"):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        
        self.trajectory = self.load_trajectory(trajectory_file)
        self.walker = None
        self.camera = None
        self.actor_list = []
        # 防止跳跃指令在同一目标点连续触发导致“飞天”
        self._last_jump_idx = -1
        self._prev_distance = None
        self._stuck_frames = 0
        # 跳跃增强：在触发后持续若干帧保持 jump=true，增强跳跃力度
        self._jump_hold_frames = 0
        self.jump_boost_z = 0.45  # 与采集一致的跳跃上抬高度
        self.record_enabled = record
        self.record_path = record_name
        self.record_writer = None
        # 可配置的可视化样式：默认 walker 保持原亮度，drone 使用更暗的颜色避免发光
        self.visual_style = visual_style
        self._colors = self._get_visual_colors(visual_style)

    def _init_video_writer(self, width=1280, height=720, fps=30.0):
        # 先尝试 mp4v，不行再尝试 avc1，若都失败则关闭录制
        for codec in ['mp4v', 'avc1']:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            writer = cv2.VideoWriter(self.record_path, fourcc, fps, (width, height))
            if writer.isOpened():
                print(f"视频录制启用，编码 {codec}, 输出 {self.record_path}")
                return writer
        print(f"视频写入器打开失败，关闭录制: {self.record_path}")
        return None
        
    def load_trajectory(self, filename):
        with open(filename, 'r') as f:
            return json.load(f)

    def spawn_playback_player(self):
        start_point = self.trajectory[0]
        spawn_loc = carla.Location(x=start_point['x'], y=start_point['y'], z=start_point['z'] + 0.5)
        spawn_transform = carla.Transform(spawn_loc)
        
        walker_bp = self.blueprint_library.filter('walker.pedestrian.*')[0]
        self.walker = self.world.spawn_actor(walker_bp, spawn_transform)
        self.actor_list.append(self.walker)
        
        # 相机跟随
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1280')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '90')
        
        camera_transform = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.walker)
        self.actor_list.append(self.camera)

    def run(self):
        if not self.trajectory:
            print("轨迹为空，退出。")
            return

        pygame.init()
        display = pygame.display.set_mode((1280, 720), pygame.HWSURFACE | pygame.DOUBLEBUF)
        
        self.spawn_playback_player()
        
        # 获取地图最高建筑高度用于绘制光柱
        all_buildings = self.world.get_environment_objects(carla.CityObjectLabel.Buildings)
        max_h = 0
        for b in all_buildings:
            h = b.bounding_box.location.z + b.bounding_box.extent.z
            if h > max_h:
                max_h = h
        self.pillar_height = max_h + 50.0

        if self.record_enabled:
            self.record_writer = self._init_video_writer(width=1280, height=720, fps=30.0)

        def process_img(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            bgr = np.array(array[:, :, :3])
            rgb = bgr[:, :, ::-1]
            if self.record_writer is not None:
                self.record_writer.write(bgr)
            surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        self.camera.listen(lambda image: process_img(image))
        
        # 预先绘制所有点和线
        self.visualize_full_trajectory()
        
        current_target_idx = 1
        clock = pygame.time.Clock()
        running = True
        
        print(f"开始回放，总计 {len(self.trajectory)} 个点...")
        
        try:
            while running and current_target_idx < len(self.trajectory):
                clock.tick(60)
                pygame.display.flip()
                
                target = self.trajectory[current_target_idx]
                target_loc = carla.Location(x=target['x'], y=target['y'], z=target['z'])
                
                current_loc = self.walker.get_location()
                direction = target_loc - current_loc
                
                # 只考虑水平方向的移动 (X, Y)，忽略 Z
                # 这可以防止小人在回放时因为 Z 轴偏差而“走上天”
                direction_2d = carla.Vector3D(x=direction.x, y=direction.y, z=0)
                distance_2d = direction_2d.length()
                # 距离阈值：普通点 0.3m，跳跃点略放宽到 0.4m，避免过早停在草丛外，又不至于过早判定到达
                arrival_threshold = 0.4 if target.get('jump', False) else 0.3
                
                control = carla.WalkerControl()
                # 若连续远离目标（距离增长），判定可能被碰撞/障碍阻挡，触发“强制到达”到下一个点
                if self._prev_distance is not None and distance_2d > self._prev_distance + 0.2:
                    self._stuck_frames += 1
                else:
                    self._stuck_frames = 0
                self._prev_distance = distance_2d

                if distance_2d > arrival_threshold:
                    # 方向：默认对准目标点；若为跳跃点且有记录的速度方向，则在目标方向基础上叠加少量记录方向以保持动量，但仍以到达目标为主
                    dir_to_target = direction_2d / distance_2d if distance_2d > 0 else carla.Vector3D(0, 0, 0)
                    recorded_dir = carla.Vector3D(x=target.get('vx', 0.0), y=target.get('vy', 0.0), z=0.0)
                    recorded_len = recorded_dir.length()
                    if target.get('jump', False) and recorded_len > 0.1:
                        blended = dir_to_target * 0.7 + (recorded_dir / recorded_len) * 0.3
                        blended_len = blended.length()
                        control.direction = blended / blended_len if blended_len > 0 else dir_to_target
                    else:
                        control.direction = dir_to_target

                    control.speed = target.get('speed', 1.5)
                    
                    # 在接近目标点（<=1.0m）时仅触发一次跳跃，并保持若干帧以增强跳跃力度
                    if target.get('jump', False) and self._last_jump_idx != current_target_idx and distance_2d <= 1.0:
                        control.jump = True
                        self._jump_hold_frames = 8  # 持续 8 帧约 0.13s
                        self._last_jump_idx = current_target_idx
                        # 额外向上轻推，增强跳跃高度（一次性），防止被低矮障碍卡住
                        try:
                            boost_loc = carla.Location(x=current_loc.x, y=current_loc.y, z=current_loc.z + self.jump_boost_z)
                            self.walker.set_location(boost_loc)
                        except Exception as _:
                            pass
                    elif self._jump_hold_frames > 0:
                        control.jump = True
                        self._jump_hold_frames -= 1
                    # 被卡住超过一定帧数且接近目标（<2.0m）则视为到达，跳过该点
                    if self._stuck_frames > 30 and distance_2d < 2.0:
                        distance_2d = arrival_threshold  # 强制满足到达条件

                if distance_2d <= arrival_threshold:
                    # 到达一个点，前往下一个
                    print(f"到达点 {current_target_idx}/{len(self.trajectory)-1}")
                    current_target_idx += 1
                    self._prev_distance = None
                    self._stuck_frames = 0
                    self._jump_hold_frames = 0
                    if current_target_idx >= len(self.trajectory):
                        print("已到达轨迹终点。")
                        break
                
                self.walker.apply_control(control)
                
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        running = False
        finally:
            print("回放结束，等待 3 秒后退出...")
            time.sleep(3)
            self.cleanup()

    def visualize_full_trajectory(self):
        # 绘制所有记录的点和虚线
        for i, p in enumerate(self.trajectory):
            loc = carla.Location(x=p['x'], y=p['y'], z=p['z'])
            # 记录点：小红球，life_time=0 表示永不消失（除非重启地图）
            self.world.debug.draw_point(loc + carla.Location(z=0.1), size=0.2, color=self._colors["point"], life_time=0)
            
            # 可视化：短促的向下箭头
            arrow_start = loc + carla.Location(z=2.5)
            arrow_end = loc + carla.Location(z=1.2)
            self.world.debug.draw_arrow(arrow_start, arrow_end, thickness=0.1, arrow_size=0.2, color=self._colors["arrow"], life_time=0)
            
            if i > 0:
                prev_p = self.trajectory[i-1]
                prev_loc = carla.Location(x=prev_p['x'], y=prev_p['y'], z=prev_p['z'] + 0.1)
                self._draw_dashed_line(prev_loc, loc + carla.Location(z=0.1))

    def _draw_dashed_line(self, start, end, dash_length=0.5):
        dist = start.distance(end)
        if dist == 0: return
        n_segments = int(dist / (dash_length * 2))
        direction = (end - start) / dist
        for i in range(n_segments):
            p1 = start + direction * (i * 2 * dash_length)
            p2 = p1 + direction * dash_length
            # life_time=0 确保连接线在回放过程中一直存在
            self.world.debug.draw_line(p1, p2, thickness=0.05, color=self._colors["line"], life_time=0)

    def _get_visual_colors(self, style):
        """Different palettes for walker (original) and drone (muted)."""
        if style == "drone":
            # 更暗、更不显眼的颜色，避免“发光”效果
            return {
                "point": carla.Color(80, 90, 120),
                "arrow": carla.Color(45, 60, 75),
                "line": carla.Color(70, 70, 80),
            }
        # 默认保持行人轨迹的原配色
        return {
            "point": carla.Color(255, 0, 0),
            "arrow": carla.Color(0, 50, 50),
            "line": carla.Color(50, 50, 0),
        }

    def cleanup(self):
        if self.camera: self.camera.stop()
        for actor in self.actor_list:
            if actor is not None:
                actor.destroy()
        if self.record_writer is not None:
            self.record_writer.release()
        pygame.quit()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("请提供轨迹文件路径: python trajectory_playback.py <trajectory_file.json> [--record output.mp4] [--style walker|drone]")
    else:
        try:
            trajectory_file = sys.argv[1]
            record = False
            record_name = None
            visual_style = "walker"
            if "--record" in sys.argv:
                record = True
                idx = sys.argv.index("--record")
                if idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith("-"):
                    record_name = sys.argv[idx + 1]
                else:
                    # 未指定文件名，则使用轨迹同名 mp4，放在相同目录
                    base, _ = os.path.splitext(trajectory_file)
                    record_name = f"{base}.mp4"
            if "--style" in sys.argv:
                style_idx = sys.argv.index("--style")
                if style_idx + 1 < len(sys.argv) and not sys.argv[style_idx + 1].startswith("-"):
                    visual_style = sys.argv[style_idx + 1]
            playback = TrajectoryPlayback(trajectory_file, record=record, record_name=record_name or "output.mp4", visual_style=visual_style)
            playback.run()
        except Exception as e:
            print(f"回放出错: {e}")


    