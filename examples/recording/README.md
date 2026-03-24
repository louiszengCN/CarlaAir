# CarlaAir Demo Recording Toolkit

录制 CarlaAir 演示视频的完整工具集。

**工作流：录制轨迹 → 导演拍摄 → 输出视频**

---

## 目录

- [环境配置](#环境配置)
- [Quick Start（5分钟上手）](#quick-start5分钟上手)
- [脚本详解](#脚本详解)
  - [record_vehicle.py — 录制车辆](#1-record_vehiclepy--录制车辆轨迹)
  - [record_drone.py — 录制无人机](#2-record_dronepy--录制无人机轨迹)
  - [record_walker.py — 录制行人](#3-record_walkerpy--录制行人轨迹)
  - [demo_director.py — 导演拍摄](#4-demo_directorpy--导演模式回放拍摄)
- [推荐 Demo 拍摄方案](#推荐-demo-拍摄方案)
- [常见问题](#常见问题)

---

## 环境配置

### 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 20.04 / 22.04 |
| GPU | NVIDIA GPU（推荐 RTX 3070 以上，至少 8GB 显存） |
| 内存 | 32GB+ |
| Python | 3.8 ~ 3.10（推荐 3.10） |
| CarlaAir | v0.1.6 打包版（已安装在本目录） |

### Step 1: 安装 Conda（如果还没有）

```bash
# 如果已经有 conda，跳过这一步
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# 重新打开终端
```

### Step 2: 创建 Python 环境

```bash
conda create -n carlaair python=3.10 -y
conda activate carlaair
```

### Step 3: 安装依赖

```bash
cd /path/to/CarlaAir-v0.1.6

# 方式 A：一键安装（推荐）
pip install -e .

# 方式 B：手动安装
pip install carla==0.9.16 airsim>=1.8.1 pygame>=2.0 numpy>=1.20 opencv-python>=4.5
```

### Step 4: 验证安装

```bash
conda activate carlaair
python -c "import carla; import airsim; import pygame; import cv2; print('All OK!')"
```

如果输出 `All OK!` 就说明环境没问题。

### Step 5: 启动 CarlaAir

```bash
cd /path/to/CarlaAir-v0.1.6
./CarlaAir.sh
```

等待终端出现类似以下信息，说明服务器准备好了：

```
[CarlaAir] CARLA server ready on port 2000
[CarlaAir] AirSim ready on port 41451
```

> **注意：** CarlaAir 需要在一个单独的终端持续运行，录制脚本在另一个终端执行。

---

## Quick Start（5分钟上手）

打开一个**新终端**，按顺序操作：

```bash
# 1. 激活环境
conda activate carlaair
cd /path/to/CarlaAir-v0.1.6

# 2. 录一段车辆轨迹（开 20~30 秒，按 F1 保存）
python examples_record_demo/record_vehicle.py

# 3. 导演模式：回放刚才的轨迹，自由摄像机拍摄
python examples_record_demo/demo_director.py trajectories/vehicle_*.json
```

导演模式里按 **F** 开始录制 MP4，拍好了再按 **F** 停止。视频保存在 `recordings/` 目录。

就这么简单！下面是详细说明。

---

## 脚本详解

### 1. record_vehicle.py — 录制车辆轨迹

驾驶一辆车，轨迹自动记录为 JSON 文件。

```bash
python examples_record_demo/record_vehicle.py
```

#### 操控键位

| 键位 | 功能 |
|------|------|
| **W** | 油门（加速） |
| **S** | 刹车 |
| **A / D** | 左转 / 右转 |
| **Space** | 手刹 |
| **R** | 切换倒车 |
| **滚轮** | 调整转向灵敏度 |
| **F1** | **保存轨迹并退出** |
| **ESC** | 退出（不保存） |

#### 可选参数

```bash
# 切换地图（支持 Town01 ~ Town10HD）
python examples_record_demo/record_vehicle.py --map Town03

# 切换天气（clear / cloudy / rain / storm / night / sunset / fog）
python examples_record_demo/record_vehicle.py --weather sunset

# 换车型（默认 Tesla Model3）
python examples_record_demo/record_vehicle.py --vehicle vehicle.audi.a2

# 指定出生点编号
python examples_record_demo/record_vehicle.py --spawn 5

# 组合使用
python examples_record_demo/record_vehicle.py --map Town05 --weather night --vehicle vehicle.bmw.grandtourer
```

#### 输出

保存到 `trajectories/vehicle_<日期时间>.json`，例如：
```
trajectories/vehicle_20260319_184504.json
```

#### 录制技巧

- 建议录 **20~40 秒**（400~800帧），太长回放时间也长
- 起步后等 2 秒再加速，避免车辆穿地
- 沿主路直行最容易拍出好效果，避免频繁急转

---

### 2. record_drone.py — 录制无人机轨迹

控制无人机飞行，记录轨迹。**异步模式运行，不会导致卡死。**

> **前提：** 需要先用 CarlaAir 的 H 菜单手动起飞无人机到想要的高度，再运行脚本。

```bash
python examples_record_demo/record_drone.py
```

#### 操控键位

| 键位 | 功能 |
|------|------|
| **W / S** | 前进 / 后退 |
| **A / D** | 左移 / 右移 |
| **E / Space** | 上升 |
| **Shift** | 下降 |
| **鼠标左右** | 偏航（转向） |
| **滚轮** | 调整飞行速度（1~25 m/s） |
| **F1** | **保存轨迹并退出** |
| **ESC** | 退出（不保存） |

#### 可选参数

```bash
# 调整飞行速度（默认 6 m/s）
python examples_record_demo/record_drone.py --speed 10

# 切换地图和天气
python examples_record_demo/record_drone.py --map Town03 --weather sunset
```

#### 输出

保存到 `trajectories/drone_<日期时间>.json`

#### 录制技巧

- 先用 H 菜单起飞到 20~50m 的高度再运行脚本
- **匀速直线飞行**效果最好，避免频繁转向
- 如果和车辆同时录，尽量**平行于车辆方向**飞行

---

### 3. record_walker.py — 录制行人轨迹

控制一个行人走路，FPS 风格操控。

```bash
python examples_record_demo/record_walker.py
```

#### 操控键位

| 键位 | 功能 |
|------|------|
| **W / S / A / D** | 前后左右走路 |
| **鼠标** | 控制看的方向 |
| **Shift** | 跑步（2.5 倍速） |
| **Space** | 跳跃 |
| **滚轮** | 调整步行速度 |
| **F1** | **保存轨迹并退出** |
| **ESC** | 退出（不保存） |

#### 可选参数

```bash
python examples_record_demo/record_walker.py --speed 3.0
python examples_record_demo/record_walker.py --map Town03 --weather rain
```

#### 输出

保存到 `trajectories/walker_<日期时间>.json`

---

### 4. demo_director.py — 导演模式（回放+拍摄）

**核心工具。** 加载之前录好的轨迹文件，所有角色同时回放，你用自由摄像机拍摄 MP4。

```bash
# 回放单条轨迹
python examples_record_demo/demo_director.py trajectories/vehicle_20260319_184504.json

# 回放所有轨迹（人+车+无人机同框）
python examples_record_demo/demo_director.py trajectories/*.json

# 指定地图、天气、高清录制
python examples_record_demo/demo_director.py --map Town03 --weather sunset --res 1920x1080 --fps 30 trajectories/*.json
```

#### 操控键位

**摄像机（FPS 风格）：**

| 键位 | 功能 |
|------|------|
| **鼠标** | 控制镜头朝向（yaw + pitch） |
| **W** | 朝你看的方向前进（含上下） |
| **S** | 后退 |
| **A / D** | 水平平移 |
| **E / Space** | 纯垂直上升 |
| **Shift / Q** | 纯垂直下降 |
| **滚轮** | 调速（0.1 ~ 100 m/s，低速精细调节） |
| **C** | 跟随角色（循环切换车/人/飞机） |
| **X** | 脱离跟随，回到自由摄像机 |

**回放控制：**

| 键位 | 功能 |
|------|------|
| **P** | 暂停 / 继续 |
| **← / →** | 前后跳 50 帧（按住 Shift = 200 帧） |
| **[ / ]** | 调整回放速度（0.25x ~ 4x） |
| **L** | 循环模式开关 |
| **Home** | 跳到开头 |

**天气控制：**

| 键位 | 天气 |
|------|------|
| **1** | 晴天 |
| **2** | 多云 |
| **3** | 小雨 |
| **4** | 暴风雨 |
| **5** | 浓雾 |
| **6** | 夜晚 |
| **7** | 日落 |
| **8** | 黎明 |
| **9** | 夜雨 |
| **0** | 自动轮播所有天气 |

**录制与其他：**

| 键位 | 功能 |
|------|------|
| **F** | 开始 / 停止录制 MP4 |
| **G** | 截图（PNG） |
| **Tab** | 开关传感器面板（RGB/Depth/Semantic） |
| **H** | 开关 HUD |
| **ESC** | 退出 |

#### 可选参数

```bash
# 高清录制
--res 1920x1080 --fps 30

# 添加背景交通（AI 自动驾驶的车辆和行人）
--traffic 20 --walkers 30

# 循环回放
--loop

# 纯导演模式（不回放轨迹，只有自由摄像机）
--no-trajectories

# 完整示例
python examples_record_demo/demo_director.py \
  --map Town03 --weather sunset \
  --res 1920x1080 --fps 30 \
  --traffic 15 --walkers 20 \
  --loop \
  trajectories/*.json
```

#### 输出

- 视频保存到 `recordings/director_<日期时间>.mp4`
- 截图保存到 `recordings/screenshot_<日期时间>.png`

---

## 推荐 Demo 拍摄方案

### Demo 1: 空地协同追踪（最经典）

展示车辆在地面行驶、无人机在空中伴飞的空地协同场景。

```bash
# Step 1: 录制车辆（沿主路直行 30 秒）
python examples_record_demo/record_vehicle.py --map Town10HD

# Step 2: 先用 H 菜单起飞无人机，然后录制（平行于道路飞行 30 秒）
python examples_record_demo/record_drone.py --map Town10HD

# Step 3: 导演拍摄（侧方 45° 俯视角，让人车飞机同框）
python examples_record_demo/demo_director.py --res 1920x1080 --fps 30 trajectories/*.json
```

### Demo 2: 天气变换

展示同一场景在不同天气下的效果。

```bash
# 录好轨迹后，导演模式里按数字键切换天气
python examples_record_demo/demo_director.py --loop trajectories/*.json
# 按 F 开始录制 → 按 1(晴天) → 等几秒 → 按 4(暴雨) → 按 6(夜晚) → 按 F 停止
```

### Demo 3: 传感器可视化

展示 CarlaAir 的多模态传感器数据。

```bash
# 录好车辆轨迹后
python examples_record_demo/demo_director.py trajectories/vehicle_*.json
# 按 Tab 打开传感器面板 → 可以看到 RGB / Depth / Semantic 三个子窗口
# 按 F 录制
```

### Demo 4: 多地图展示

展示不同城市环境。

```bash
# Town01 小镇
python examples_record_demo/record_vehicle.py --map Town01
python examples_record_demo/demo_director.py --map Town01 trajectories/vehicle_*.json

# Town03 郊区
python examples_record_demo/record_vehicle.py --map Town03
python examples_record_demo/demo_director.py --map Town03 --weather sunset trajectories/vehicle_*.json

# Town05 城市
python examples_record_demo/record_vehicle.py --map Town05
python examples_record_demo/demo_director.py --map Town05 --weather night trajectories/vehicle_*.json
```

### Demo 5: 大规模交通场景

```bash
# 不需要录轨迹，直接用背景交通
python examples_record_demo/demo_director.py \
  --no-trajectories --traffic 30 --walkers 50 \
  --map Town10HD --res 1920x1080 --fps 30
# 用自由摄像机飞来飞去拍摄城市交通
```

---

## 目录结构

```
CarlaAir-v0.1.6/
├── CarlaAir.sh               # CarlaAir 启动脚本
├── examples_record_demo/     # ← 你在这里
│   ├── README.md             # 本文件
│   ├── record_vehicle.py     # 录制车辆轨迹
│   ├── record_drone.py       # 录制无人机轨迹
│   ├── record_walker.py      # 录制行人轨迹
│   └── demo_director.py      # 导演模式（回放 + 拍摄）
├── trajectories/             # 轨迹文件（自动创建）
│   ├── vehicle_20260319_184504.json
│   ├── drone_20260319_185012.json
│   └── walker_20260319_185530.json
└── recordings/               # 视频和截图（自动创建）
    ├── director_20260319_190000.mp4
    └── screenshot_20260319_190030.png
```

---

## 常见问题

### Q: 运行脚本报 `ModuleNotFoundError: No module named 'carla'`

确保已激活正确的 conda 环境：
```bash
conda activate carlaair  # 或你安装了依赖的环境名
```

### Q: 提示 `Version mismatch`（Client 0.9.16 vs Simulator `1ae5356-dirty`）或 `std::bad_alloc` 崩溃

CarlaAir 的仿真器是 **CARLA 定制分支**，不能与 PyPI 上的 `carla==0.9.16` 客户端混用，否则会 RPC 不兼容并可能 `std::bad_alloc`。

1. 向发布方索取与当前 CarlaAir **同一次编译** 的 Python wheel（`carla-*.whl`），或从完整 `LinuxNoEditor` 包里的 `PythonAPI/carla/dist/` 复制。
2. 将 **单个** `carla-*.whl` 放到项目目录：  
   `CarlaAir-v0.1.6/PythonAPI/carla/dist/`
3. 在项目根目录执行：

```bash
./install_matching_carla_api.sh
# 或指定路径： ./install_matching_carla_api.sh /path/to/carla-....whl
```

再运行录制脚本。若 `PythonAPI/carla/dist/` 为空，说明当前压缩包可能漏带了 wheel，需要补发或自行用源码树 `Util/BuildTools/BuildPythonAPI.sh` 在匹配环境下编译生成。

### Q: 运行脚本报 `time-out while waiting for the simulator`

CarlaAir 没有在运行。在另一个终端执行：
```bash
cd /path/to/CarlaAir-v0.1.6
./CarlaAir.sh
```

### Q: 无人机脚本打开后无法控制 / 没有画面

1. 确保已经通过 H 菜单起飞了无人机
2. 如果没有画面但能控制，说明 CARLA 找不到 drone actor，检查 AirSim 是否正常连接

### Q: 导演模式里速度太快或太慢

滚轮可以调速，范围是 **0.1 ~ 100 m/s**：
- 低速区（< 2 m/s）：每滚一格 ±0.2
- 中速区（2~10 m/s）：每滚一格 ±1.0
- 高速区（> 10 m/s）：每滚一格 ±5.0

### Q: 轨迹回放时角色位置不对

确保录制和回放使用的是**同一张地图**。轨迹文件里记录了地图名，切换地图后坐标会不对。

### Q: 导演模式的回放结束了怎么办

- 按 **P** 暂停后按 **Home** 跳到开头，再按 **P** 继续
- 或者启动时加 `--loop` 循环播放

### Q: 怎么让录制的视频更流畅

```bash
# 使用 30fps + 1080p
python examples_record_demo/demo_director.py --res 1920x1080 --fps 30 trajectories/*.json
```

### Q: 车辆出生后掉到地下

重新录制，起步后**等 2~3 秒**让物理引擎稳定再开始操作。

### Q: 我想同时录多辆车 / 多个行人

每种类型运行多次录制脚本即可，每次都会生成独立的 JSON 文件。导演模式用 `trajectories/*.json` 可以同时加载所有轨迹。
