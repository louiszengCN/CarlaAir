      
import carla
import json
import time
import random
import argparse
import numpy as np
import cv2
import pygame
from pygame.locals import K_w, K_s, K_a, K_d, K_r, K_q, K_t, K_ESCAPE, K_SPACE, K_RETURN, K_c

class TrajectoryCollector:
    def __init__(self, debug_life_time: float = 8.0):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.map = self.world.get_map()
        
        self.walker = None
        self.camera = None
        self.recorded_points = []
        self.visual_points = []  # 仅用于可视化连接，不影响保存的数据
        self.debug_points = []
        self.debug_arrows = []
        self.debug_dashes = []
        self.actor_list = []
        
        # 状态变量
        self.is_fps = True
        self.move_speed = 1.5
        self.look_yaw = 0.0
        self.look_pitch = 0.0
        self.mouse_sensitivity = 0.1
        self.jump_requested = False # 用于追踪是否按下了跳跃
        self.jump_boost_z = 0.45  # 跳跃增强高度，用于跨越常见矮障碍

        # debug 可视化参数：避免 life_time=0 造成持久化绘制累计，拖慢仿真器
        self.debug_life_time = float(debug_life_time)  # 秒；越大越“持久”，但也越可能拖慢
        
        # 获取地图最高建筑高度
        self.max_height = self._get_max_building_height()
        self.pillar_height = self.max_height + 50.0 
        
    def _get_max_building_height(self):
        all_buildings = self.world.get_environment_objects(carla.CityObjectLabel.Buildings)
        max_h = 0
        for b in all_buildings:
            h = b.bounding_box.location.z + b.bounding_box.extent.z
            if h > max_h:
                max_h = h
        return max_h

    def spawn_player(self):
        spawn_point = self._get_spawn_point()
        self._spawn_walker_and_camera(spawn_point)
        print(f"行人已生成，当前为 {'第一人称 FPS' if self.is_fps else '第三人称'} 模式。速度: {self.move_speed:.1f}m/s")

    def _get_spawn_point(self):
        """获取用于出生的 transform。"""
        spawn_loc = self.world.get_random_location_from_navigation()
        if spawn_loc is not None:
            return carla.Transform(spawn_loc + carla.Location(z=0.5))
        spawn_points = self.map.get_spawn_points()
        base_point = random.choice(spawn_points) if spawn_points else carla.Transform(carla.Location(x=0, y=0, z=2))
        waypoint = self.map.get_waypoint(base_point.location, lane_type=carla.LaneType.Sidewalk)
        if waypoint:
            spawn_point = waypoint.transform
            spawn_point.location.z += 0.5
            return spawn_point
        return base_point

    def _spawn_walker_and_camera(self, spawn_point):
        """在给定位置生成行人与相机。"""
        walker_bp = self.blueprint_library.filter('walker.pedestrian.*')[0]
        self.walker = self.world.spawn_actor(walker_bp, spawn_point)
        self.actor_list.append(self.walker)

        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1280')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '100')

        self.camera = self.world.spawn_actor(camera_bp, carla.Transform(), attach_to=self.walker)
        self.actor_list.append(self.camera)

        self.look_yaw = spawn_point.rotation.yaw
        self.update_camera_mode()

    def update_camera_mode(self):
        if self.is_fps:
            # 第一人称：眼睛位置
            self.camera.set_transform(carla.Transform(carla.Location(x=0.1, z=1.7)))
        else:
            # 第三人称：后上方
            self.camera.set_transform(carla.Transform(carla.Location(x=-3.5, z=2.5), carla.Rotation(pitch=-15)))

    def record_point(self):
        loc = self.walker.get_location()
        # 记录当前移动速度标量与速度矢量，便于回放时还原方向和大小
        current_speed = self.move_speed
        vel = self.walker.get_velocity()
        # 记录位置和跳跃状态
        point = {
            "x": loc.x, 
            "y": loc.y, 
            "z": loc.z, 
            "jump": self.jump_requested,
            "speed": current_speed,
            "vx": vel.x,
            "vy": vel.y,
            "vz": vel.z
        }
        self.recorded_points.append(point)
        print(f"已记录第 {len(self.recorded_points)} 个点: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
        
        # 重置跳跃状态
        self.jump_requested = False
        
        # 可视化（仅在记录点时绘制一次，避免逐帧重绘导致卡顿）
        self.world.debug.draw_point(
            loc + carla.Location(z=0.1),
            size=0.2,
            color=carla.Color(255, 0, 0),
            life_time=self.debug_life_time,
            persistent_lines=False,
        )

        arrow_start = loc + carla.Location(z=2.5)
        arrow_end = loc + carla.Location(z=1.2)
        self.world.debug.draw_arrow(
            arrow_start,
            arrow_end,
            thickness=0.1,
            arrow_size=0.2,
            color=carla.Color(0, 50, 50),
            life_time=self.debug_life_time,
            persistent_lines=False,
        )

        if len(self.recorded_points) > 1:
            prev_p = self.recorded_points[-2]
            prev_loc = carla.Location(x=prev_p['x'], y=prev_p['y'], z=prev_p['z'] + 0.1)
            self._draw_connection_line(prev_loc, loc + carla.Location(z=0.1))

    def _draw_connection_line(self, start, end):
        """用单条线连接相邻点（比虚线分段 draw_line 更省，避免拖慢仿真器）。"""
        try:
            self.world.debug.draw_line(
                start,
                end,
                thickness=0.05,
                color=carla.Color(50, 50, 0),
                life_time=self.debug_life_time,
                persistent_lines=False,
            )
        except Exception:
            pass

    def _destroy_actors(self):
        if self.camera:
            try:
                self.camera.stop()
            except Exception:
                pass
        for actor in self.actor_list:
            if actor is not None:
                try:
                    actor.destroy()
                except Exception:
                    pass
        self.actor_list = []

    def run(self):
        pygame.init()
        display = pygame.display.set_mode((1280, 720), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.event.set_grab(True) # 锁定鼠标
        pygame.mouse.set_visible(False) # 隐藏鼠标
        
        self.spawn_player()
        
        # 相机回调
        def process_img(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1] 
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        self.camera.listen(lambda image: process_img(image))
        
        clock = pygame.time.Clock()
        running = True
        
        print("\nFPS 模式控制说明:")
        print("  WASD: 移动")
        print("  鼠标: 转动视角")
        print("  鼠标滚轮: 调节移动速度")
        print("  T: 切换第一/第三人称")
        print("  SPACE: 跳跃")
        print("  R: 记录当前位置点")
        print("  C: 清除地图上已绘制的轨迹")
        print("  Q: 保存当前轨迹并开始新记录")
        print("  ESC: 退出程序")
        
        while running:
            clock.tick(60)
            pygame.display.flip()
            if self.walker is None or self.camera is None:
                try:
                    self.spawn_player()
                    continue
                except Exception:
                    running = False
                    continue
            
            # 1. 鼠标处理 (旋转)
            dx, dy = pygame.mouse.get_rel()
            self.look_yaw += dx * self.mouse_sensitivity
            self.look_pitch = np.clip(self.look_pitch - dy * self.mouse_sensitivity, -60, 60)
            
            # 更新行人朝向 (仅 Yaw)
            walker_transform = self.walker.get_transform()
            walker_transform.rotation.yaw = self.look_yaw
            self.walker.set_transform(walker_transform)
            
            # 更新相机旋转
            if self.is_fps:
                self.camera.set_transform(carla.Transform(
                    carla.Location(x=0.1, z=1.7), 
                    carla.Rotation(pitch=self.look_pitch, yaw=0, roll=0)
                ))
            else:
                self.camera.set_transform(carla.Transform(
                    carla.Location(x=-3.5, z=2.5), 
                    carla.Rotation(pitch=self.look_pitch - 15, yaw=0, roll=0)
                ))
            
            # 2. 键盘处理 (移动)
            keys = pygame.key.get_pressed()
            control = carla.WalkerControl()
            
            direction = carla.Vector3D(0, 0, 0)
            if keys[K_w]: direction.x = 1
            if keys[K_s]: direction.x = -1
            if keys[K_a]: direction.y = -1
            if keys[K_d]: direction.y = 1
            
            if direction.length() > 0:
                yaw_rad = np.radians(self.look_yaw)
                world_dir = carla.Vector3D()
                world_dir.x = direction.x * np.cos(yaw_rad) - direction.y * np.sin(yaw_rad)
                world_dir.y = direction.x * np.sin(yaw_rad) + direction.y * np.cos(yaw_rad)
                control.direction = world_dir
                control.speed = self.move_speed
            
            # 3. 事件处理
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_SPACE:
                        self.jump_requested = True # 标记为已跳跃
                        control.jump = True
                        self.record_point() # 按下跳跃时立即自动记录该点
                        # 采集时同步轻推角色向上，增强跳跃能力
                        try:
                            boost_loc = self.walker.get_location()
                            boost_loc.z += self.jump_boost_z
                            self.walker.set_location(boost_loc)
                        except Exception:
                            pass
                    elif event.key == K_r:
                        self.record_point()
                    elif event.key == K_t:
                        self.is_fps = not self.is_fps
                        self.update_camera_mode()
                        print(f"切换至 {'第一人称' if self.is_fps else '第三人称'} 视图")
                    elif event.key == K_c:
                        self.clear_drawings()
                    elif event.key == K_q:
                        self.save_trajectory()
                        self.recorded_points = []
                        print("已保存并开始新记录。")
                    elif event.key == K_ESCAPE:
                        running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4: # Scroll Up
                        self.move_speed = min(self.move_speed + 0.5, 10.0)
                        print(f"移动速度增加: {self.move_speed:.1f}m/s")
                    elif event.button == 5: # Scroll Down
                        self.move_speed = max(self.move_speed - 0.5, 0.5)
                        print(f"移动速度减小: {self.move_speed:.1f}m/s")
            
            self.walker.apply_control(control)
        
        self.cleanup()

    def save_trajectory(self):
        if not self.recorded_points:
            print("没有记录任何轨迹点，跳过保存。")
            return
        
        filename = f"trajectory_{int(time.time())}.json"
        with open(filename, 'w') as f:
            json.dump(self.recorded_points, f, indent=4)
        print(f"轨迹已保存至 {filename}, 共 {len(self.recorded_points)} 个点。")

    def cleanup(self):
        print("清理资源...")
        pygame.event.set_grab(False)
        pygame.mouse.set_visible(True)
        self._destroy_actors()
        pygame.quit()

    def clear_drawings(self):
        """清除已绘制的轨迹（仅可视化，不影响已记录的点数据）。"""
        # 保存当前位置与朝向，用于重生
        try:
            current_tf = self.walker.get_transform() if self.walker else None
        except Exception:
            current_tf = None

        # 清空可视化缓存
        self.debug_points.clear()
        self.debug_arrows.clear()
        self.debug_dashes.clear()
        self.visual_points.clear()

        # 重新加载地图以清除持久化 debug 绘制
        try:
            raw_map_name = self.world.get_map().name
            base_map_name = raw_map_name.split('/')[-1] if raw_map_name else raw_map_name
            print("正在重新加载地图以清除轨迹绘制...")
            self._destroy_actors()
            self.walker = None
            self.camera = None
            try:
                # 优先使用 reload_world（更通用，也更快）
                self.world = self.client.reload_world()
            except Exception:
                self.world = self.client.load_world(base_map_name)
            self.map = self.world.get_map()
            self.blueprint_library = self.world.get_blueprint_library()

            # 根据原位置重生行人；如果失败则回退随机出生
            if current_tf is not None:
                try:
                    self._spawn_walker_and_camera(current_tf)
                except Exception:
                    self.spawn_player()
            else:
                self.spawn_player()
            print("轨迹已清除并在原位置重生（如无效则随机重生）。")
        except Exception as e:
            print(f"重新加载地图清除轨迹失败: {e}")
            try:
                # 尝试恢复当前 world 并重生
                self.world = self.client.get_world()
                self.map = self.world.get_map()
                self.blueprint_library = self.world.get_blueprint_library()
                if self.walker is None or self.camera is None:
                    self.spawn_player()
            except Exception as e2:
                print(f"恢复世界失败: {e2}")

if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--time", type=float, default=8.0, help="轨迹可视化显示时长（秒）")
        args = parser.parse_args()
        collector = TrajectoryCollector(debug_life_time=args.time)
        collector.run()
    except Exception as e:
        print(f"运行出错: {e}")



    