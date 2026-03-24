# CarlaAir Quick-Start Guide / 快速入门指南

欢迎使用 **CarlaAir** — 空地一体联合仿真平台。
本指南帮助你从零开始，10 分钟内完成首次体验。

---

## Prerequisites / 前置条件

在开始之前，请确认以下环境已准备好：

- **Conda** — 已安装 Miniconda 或 Anaconda
- **CarlaAir 二进制包** — 已下载并解压 CarlaAir 发行包
- **NVIDIA GPU** — 至少 8GB 显存（Town10HD 需要约 8GB）
- **Ubuntu 系统** — 推荐 Ubuntu 20.04 / 22.04

---

## Environment Setup / 环境配置

使用自动化脚本一键配置 Python 环境和依赖：

```bash
# 创建 conda 环境并安装所有依赖
bash env_setup/setup_env.sh

# 验证环境是否正确配置
bash env_setup/test_env.sh
```

`setup_env.sh` 会创建 `carlaAir` conda 环境，安装 CARLA Python API、AirSim、PyGame、OpenCV 等依赖。
`test_env.sh` 会检查所有依赖是否可导入。

配置完成后，激活环境：

```bash
conda activate carlaAir
```

---

## Launch / 启动仿真器

```bash
./CarlaAir.sh Town10HD
```

等待看到 "Ready! Both servers are running." 即表示启动成功。

**端口说明 / Ports:**
| 服务 Service | 端口 Port | 用途 Purpose |
|---|---|---|
| CARLA API | `localhost:2000` | 地面仿真：车辆、行人、天气、传感器 |
| AirSim API | `localhost:41451` | 空中仿真：无人机飞行与相机 |

> **提示**: 首次启动需要 2-5 分钟加载资源和编译着色器，请耐心等待。

---

## First Experience / 首次体验

启动成功后，运行你的第一个 CarlaAir 脚本：

```bash
python3 examples/quick_start_showcase.py
```

这个脚本会自动演示 CarlaAir 的核心能力：
1. 在城市中生成一辆 Tesla，无人机起飞并锁定跟踪
2. 车辆沿道路行驶，无人机在上方自动跟随
3. 4 分屏传感器展示：RGB 画面 / 深度图 / 语义分割 / 无人机 FPV
4. 天气自动循环切换（晴天、雨天、雾天、日落等）

**操作 / Controls:** `N` 手动切换天气，`ESC` 退出。

---

## Interactive Demos / 交互式示例

`examples/` 目录包含 6 个经过测试的示例脚本，涵盖 CarlaAir 的各项能力：

| 脚本 Script | 功能 Description | 操作 Controls |
|---|---|---|
| `quick_start_showcase.py` | 全自动演示：车辆+无人机跟踪+4分屏传感器+天气循环 | `N` 切换天气, `ESC` 退出 |
| `drive_vehicle.py` | 键盘驾驶车辆，第三人称追踪相机 | `WASD` 驾驶, `Space` 手刹, `R` 倒车, `N` 天气 |
| `walk_pedestrian.py` | 第一人称行人漫游，鼠标控制视角 | `WASD` 行走, `Shift` 加速, `Space` 跳跃, `Mouse` 视角 |
| `switch_maps.py` | 自动遍历所有地图，每张地图无人机飞越展示 | `N` 跳过当前地图, `ESC` 退出 |
| `sensor_gallery.py` | 6 格传感器展示：RGB/深度/语义/实例/LiDAR/光流 | `N` 切换天气, `ESC` 退出 |
| `air_ground_sync.py` | 左右分屏：地面车辆+空中无人机，同一世界同步天气 | `N` 切换天气, `ESC` 退出 |

运行方式 / Usage:

```bash
conda activate carlaAir
python3 examples/drive_vehicle.py
```

---

## Recording Toolkit / 录制工具

`examples_record_demo/` 目录提供了轨迹录制与回放工具，用于制作高质量演示视频：

| 脚本 Script | 用途 Purpose |
|---|---|
| `record_vehicle.py` | 录制车辆行驶轨迹（键盘操控 + 轨迹保存） |
| `record_drone.py` | 录制无人机飞行轨迹（键盘操控 + 轨迹保存） |
| `record_walker.py` | 录制行人行走轨迹 |
| `demo_director.py` | 读取轨迹文件，同步回放并录制高质量视频帧 |
| `trajectory_helpers.py` | 轨迹文件的读写工具函数 |

**工作流 / Workflow:**
1. 用 `record_*.py` 脚本交互录制轨迹（保存为 JSON 文件）
2. 用 `demo_director.py` 加载轨迹并以同步模式回放，输出帧序列
3. 用 ffmpeg 将帧序列合成视频

详见 `examples_record_demo/README.md`。

---

## Synchronous vs Asynchronous Mode / 同步与异步模式

CarlaAir 支持两种运行模式，不同场景下需要选择合适的模式：

### Asynchronous Mode / 异步模式（默认）

适用于**交互式操作**：在 CarlaAir 窗口中驾驶车辆、操控无人机。

- 仿真器按自身节奏推进，客户端随时读取最新状态
- 帧率不固定，画面流畅但时间步不均匀
- 适合：键盘驾驶、实时飞行、手动探索

### Synchronous Mode / 同步模式

适用于**数据采集、演示回放、视频录制**等需要精确控制的场景。

- 仿真器每帧等待客户端调用 `world.tick()` 后才推进
- 时间步固定，帧帧一致，适合录制平滑视频

**同步模式配置要求 / Configuration:**

```python
# 开启同步模式
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 1.0 / 30  # 固定 30 FPS
world.apply_settings(settings)

# Traffic Manager 也需要同步
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)

# 主循环中每帧调用
world.tick()
```

> **重要提示**: 脚本退出时**必须恢复异步模式**，否则仿真器会冻结：
>
> ```python
> settings = world.get_settings()
> settings.synchronous_mode = False
> settings.fixed_delta_seconds = None
> world.apply_settings(settings)
> ```

---

## Coordinate Systems / 坐标系统

CarlaAir 中有两套坐标系统并存，编写脚本时需注意转换：

| | CARLA | AirSim (NED) |
|---|---|---|
| X | 向前 (East) | 向前 (North) |
| Y | 向右 | 向右 (East) |
| Z | **向上为正** | **向下为正** |

**关键换算**: AirSim NED z = -CARLA z。例如 CARLA 中 30 米高空 = AirSim NED z = -30。

两套 API 的世界原点不同，具体换算关系详见：
`examples_record_demo/COORDINATE_SYSTEMS.md`

---

## Available Maps / 可用地图

CarlaAir 内置 6 张城市地图：

| 地图 Map | 描述 Description |
|---|---|
| **Town01** | 小镇，T 字路口为主，简单路网 |
| **Town02** | 小镇，住宅区风格 |
| **Town03** | 较大城区，环岛和隧道 |
| **Town04** | 高速公路 + 小镇，8 字形高架桥 |
| **Town05** | 多车道城市，方格路网 |
| **Town10HD** | 高清滨海城市（默认地图），最丰富的场景 |

启动时指定地图 / Launch with specific map:

```bash
./CarlaAir.sh Town03
```

> **关于 _Opt 变体**: 从 CARLA 官方包中可以获取 `Town01_Opt` 等优化版地图。`_Opt` 变体使用分层加载（Level Streaming），减少显存占用，适合低配 GPU。

---

## API Reference / API 参考

### CARLA Python API (port 2000)

核心类 / Core classes:
- `carla.Client` — 连接仿真器
- `carla.World` — 世界管理（天气、Actor、传感器）
- `carla.Actor` — 车辆、行人、传感器等实体
- `carla.Transform` / `carla.Location` — 位姿与坐标

官方文档 / Official docs: https://carla.readthedocs.io/en/0.9.16/

### AirSim Python API (port 41451)

核心类 / Core classes:
- `airsim.MultirotorClient` — 无人机控制
- `airsim.ImageRequest` — 图像采集请求

官方文档 / Official docs: https://microsoft.github.io/AirSim/

---

## FAQ / Troubleshooting / 常见问题

### 仿真器启动相关

| 问题 Problem | 解决方案 Solution |
|---|---|
| CARLA 连接超时 | 首次启动需要 2-5 分钟加载资源，请耐心等待。用 `ss -tlnp \| grep 2000` 检查端口 |
| AirSim 连接失败 | AirSim 在 CARLA 之后启动，需额外等待。确认 `~/Documents/AirSim/settings.json` 存在 |
| GPU 显存不足 | Town10HD 需约 8GB 显存。尝试更小地图（Town01）或添加 `-quality-level=Low` |
| 帧率低 | 减少 Actor 数量，降低窗口分辨率，使用较小地图 |

### 脚本运行相关

| 问题 Problem | 解决方案 Solution |
|---|---|
| 无人机不响应指令 | 确保调用了 `enableApiControl(True)` + `armDisarm(True)` + `takeoffAsync().join()` |
| Walker AI Controller 崩溃 | 已知限制，不要对 Walker 调用 `go_to_location()`，仅使用静态 Walker |
| 10+ 车辆 + 无人机崩溃 | 使用 `try_spawn_actor()` 并限制 autopilot 车辆 ≤ 8 辆 |
| 同步模式下仿真器冻结 | 脚本退出前必须恢复异步模式（见上方同步模式章节） |
| `simSetCameraPose` 崩溃 | 在 Shipping 构建中此函数会触发 C++ abort，不要调用。用无人机 yaw + 轨道飞行替代 |
| 车辆行驶异常/抽搐 | 使用 `hybrid_physics_mode` 启用混合物理模式 |
| AirSim 客户端干扰无人机控制 | 使用 `examples_record_demo/record_drone.py` 进行无人机录制 |

### 坐标与地图相关

| 问题 Problem | 解决方案 Solution |
|---|---|
| 无人机位置和 CARLA 车辆位置对不上 | 两套 API 的坐标原点和 Z 轴方向不同，详见 COORDINATE_SYSTEMS.md |
| 如何切换地图 | 启动时指定：`./CarlaAir.sh Town03` |
| 天气变化是否影响无人机 | 是的，`set_weather()` 同时影响地面和空中视角 |
