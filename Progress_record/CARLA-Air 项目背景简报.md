# CarlaAir 项目背景简报

**文档用途**：供技术报告撰写 AI Agent 快速理解项目全貌、技术细节与当前进度，作为撰写正式技术报告（arXiv paper）的背景输入材料
**项目名称**：CarlaAir（有时写作 CARLA-Air，正式名称为 CarlaAir）
**当前版本**：v0.1.5
**日期**：2026-03-18
**作者**：南方科技大学

---

## 一、项目概述

### 1.1 一句话定义

**CarlaAir** 是一个面向机器人与自动驾驶研究的**空地联合仿真平台（Ground-Aerial Co-simulation Platform）**，将 CARLA（地面自动驾驶仿真器）和 AirSim（无人机仿真器）合并进同一个 Unreal Engine 4 进程中运行，实现传感器数据帧级同步。

### 1.2 核心价值

CarlaAir 让研究者第一次可以在同一个虚拟城市里，同时操控地面车辆和空中无人机，两者的传感器数据完全同步、天气完全一致、物理引擎完全共享。这是通过 **in-process integration**（进程内集成）实现的，而非传统的 bridge-based 跨进程通信方案。

### 1.3 解决的问题

| 现有方案 | 局限 |
|----------|------|
| CARLA 单独使用 | 全球最流行的自动驾驶仿真器，有逼真城市、车辆、行人，但**没有无人机** |
| AirSim 单独使用 | 最流行的无人机仿真器，有精确飞行物理，但**没有城市交通和行人** |
| Bridge-based 方案（CARLA + AirSim 通过网络通信） | 两个独立进程，画面/天气/时间无法精确同步，延迟不可控 |
| **CarlaAir（本项目）** | **单进程集成**，共享 UE4 世界、物理引擎、渲染管线，数据天然同步 |

### 1.4 核心技术原理

两个仿真器都基于 Unreal Engine 4，但 UE4 有一个限制：每个虚拟世界只能有一个 `GameMode` 控制器。CARLA 有自己的 `GameMode`，AirSim 也有自己的 `GameMode`，两者无法共存。

CarlaAir 的核心技术贡献是设计了一个统一控制器 `ASimWorldGameMode`：
- CARLA 的 `GameMode` 作为**主控**
- AirSim 的控制逻辑作为**附属组件（Component）**挂载进来
- 只需修改 CARLA 源代码中的两个文件，改动极小
- 效果：两个仿真器在同一进程中完全融合运行

---

## 二、平台能力详述

### 2.1 仿真世界规模

| 能力 | 数量/规格 |
|------|----------|
| 城市地图 | 13 张（Town01-Town07、Town10HD 等，均为 CARLA 官方地图） |
| 天气预设 | 14 种（晴天、多云、雨天、暴雨、雾天等，15 个参数可独立调节） |
| 地面角色蓝图 | 220+ 种（41 种车型、52 种行人形象、各类道具） |
| 无人机 | 1 架（默认，可配置多架） |
| 同时运行规模 | 30 辆地面车辆 + 1 架 AirSim 无人机，验证稳定 |

### 2.2 传感器能力

地面车辆和空中无人机各自携带一套完整的传感器，共 **18 路同步传感器流**：

| 平台 | 传感器类型 |
|------|-----------|
| 地面（CARLA） | RGB 相机、深度相机、语义分割、实例分割、LiDAR 激光雷达、毫米波雷达、IMU、GPS、碰撞传感器、车道入侵传感器 |
| 空中（AirSim） | RGB 相机、深度相机、语义分割、红外相机、IMU、GPS、气压计、磁力计 |

**传感器并发稳定性（已验证）**：6 个传感器（3 Camera + LiDAR + IMU + GNSS）同时运行 10 分钟稳定，各传感器帧数记录：

| 传感器组 | 10分钟帧数 |
|----------|-----------|
| Camera 组 | 28,986 帧 |
| LiDAR + IMU | 24,866 帧 |
| GNSS | 59,053 帧 |

### 2.3 编程接口（Dual API）

研究者通过 **Python** 控制整个仿真世界。两套 API 可以在同一个 Python 脚本里同时使用：

```python
# 同时使用 CARLA API 和 AirSim API
carla_client = carla.Client('localhost', 2000)   # 连接 CARLA（地面仿真）
airsim_client = airsim.MultirotorClient()        # 连接 AirSim（无人机仿真）

# 在同一个脚本里同时控制两者
carla_client.spawn_vehicle(...)      # 在城市里生成一辆车
airsim_client.takeoffAsync()         # 无人机起飞
# 两者在同一个虚拟世界里，完全同步
```

**API 测试结果**：89/89 全部通过（PASS）。

### 2.4 内置 FPS 无人机控制系统（v0.1.2 新增）

从 v0.1.2 起，CarlaAir 内置了 C++ 实现的第一人称（FPS）无人机操控系统，**无需 Python 脚本**即可直接在仿真器内控制无人机飞行：

| 控制 | 按键 |
|------|------|
| 水平移动 | WASD |
| 偏航（Yaw） | 鼠标左右 |
| 上升 | Space |
| 下降 | Shift |
| 速度调节 | 鼠标滚轮（v0.1.5 起，替代了早期的 ]/[ 按键） |
| 碰撞模式切换 | P 键（物理碰撞 / 无敌穿透模式） |
| 帮助菜单 | H 键（中英双语） |

这个内置控制系统的技术意义：
- 降低了使用门槛——不需要写 Python 脚本就能试飞无人机
- 用于 Demo 录制和快速测试
- 完全在 C++/UE4 层实现，不经过 Python，性能最优

### 2.5 碰撞物理系统（v0.1.2-v0.1.5 迭代）

碰撞检测系统经历了多次迭代，最终方案如下：

| 版本 | 碰撞方案 | 问题 |
|------|----------|------|
| v0.1.1 及之前 | UE4 Sweep 碰撞检测 | CARLA 地图中存在大量 false sweep collision，无人机无法正常飞行 |
| v0.1.2 | Sweep→Teleport + Ground Clamp（Line Trace）+ Sphere Overlap Bounce（1m 半径检测 + 1.2m 反弹） | 修复了无人机飞行问题，但仍有边界情况 |
| v0.1.4 | 增强碰撞检测（1m Sphere + 1.2m Bounce + AirSim API 碰撞追踪） | 基本稳定 |
| **v0.1.5（当前）** | **12-direction LineTrace 系统**（8 水平方向 + 4 对角向下）替代 Sphere Overlap | 最终方案，精确度和性能最优 |

v0.1.5 还新增了 **P 键切换模式**：
- **物理碰撞模式**：12 方向 LineTrace 碰撞检测，撞到障碍物会被弹开
- **无敌穿透模式**：关闭碰撞，可穿越任何建筑物，适合快速到达目标位置

### 2.6 坐标系统对齐

CarlaAir 实现了 CARLA 坐标系和 AirSim 坐标系的**精确对齐**，误差为 **0.0000m**。

这是一个关键技术成就——两个仿真器原本使用不同的坐标系统（CARLA 使用左手坐标系，AirSim 使用 NED 坐标系），CarlaAir 在引擎层面实现了精确转换，确保无人机和地面车辆的空间关系完全正确。

**已修复的坐标相关问题**：
- 车辆穿越地面问题——由 `SetNewWorldOrigin` 导致，已禁用并修复
- 无人机已作为 `airsim.drone` 注册到 CARLA 的 `ActorDispatcher`，可通过 `world.get_actors()` 查询到

### 2.7 ROS 2 支持

平台原生支持 **ROS 2**（机器人操作系统），已验证的 ROS 话题数量：

| 类别 | 话题数 |
|------|--------|
| CARLA ROS Bridge 话题 | 43 |
| AirSim ROS Bridge 话题 | 14 |
| 通用话题 | 6 |
| **总计** | **63** |

- 双 Bridge 同时运行（CARLA ROS Bridge + AirSim ROS Bridge），63 个话题同时发布
- rviz2 可视化已确认可用
- 第二轮专家测试中验证了 39 个 ROS 话题的数据正确性

### 2.8 帮助菜单系统（v0.1.5）

H 键呼出的帮助菜单经历了三次设计迭代：

1. **v1**：GEngine 调试信息输出（UE4 原生调试文字）
2. **v2**：Slate Widget 实现（Apple HIG 风格）
3. **v3（当前）**：纯黑背景（α=0.92），大号粗体字，中英文双语显示

---

## 三、目标用户

| 用户群体 | 使用场景 |
|----------|----------|
| 自动驾驶研究者 | 使用 CARLA 做自动驾驶算法，引入无人机视角做空地协同感知 |
| 无人机研究者 | 使用 AirSim 做无人机算法，在真实城市交通场景中测试 |
| ROS 开发者 | 直接订阅 CarlaAir 传感器话题，发布控制指令 |
| 数据集构建者 | 大规模、多模态、空地联合仿真数据集采集 |
| 具身智能研究者 | 无人机在有行人的城市环境中的社会导航 |

活跃社区：arXiv、Hugging Face、GitHub、Reddit（r/MachineLearning、r/robotics）

---

## 四、版本迭代历史

### 4.1 版本时间线

| 版本 | 日期 | 关键变更 |
|------|------|----------|
| v0.1.1 及之前 | 2026-03-12 之前 | 基础集成完成，API 通过测试 |
| **v0.1.2** | 2026-03-13 | 内置 FPS 无人机控制、Sweep→Teleport 碰撞修复、坐标精确对齐、CARLA ActorDispatcher 注册、CarlaAir.sh 改进 |
| **v0.1.3** | 2026-03-13 | FPSYaw_ 初始化修复（W 键方向 bug）、默认第三人称视角（移除 V 键切换） |
| **v0.1.4** | 2026-03-13 ~ 03-16 | 减号键黑屏修复、碰撞检测增强、ROS2 适配验证（63 话题）、第一次正式发布包、5 个轨迹录制脚本、setup.py + README.md |
| **v0.1.5** | 2026-03-16 ~ 03-18 | 碰撞系统重写（12-direction LineTrace）、鼠标滚轮速度调节、P 键碰撞模式切换、H 键双语帮助菜单 |

### 4.2 v0.1.2 详细变更

- **内置 C++ FPS 无人机控制**：WASD 移动 + 鼠标偏航 + Space/Shift 垂直升降，完全在 C++ 层实现，无需 Python
- **Sweep→Teleport 碰撞修复**：CARLA 地图中存在大量 false sweep collision（虚假碰撞），导致无人机根本无法飞行；解决方案是将移动方式从 Sweep 改为 Teleport
- **Ground Clamp（Line Trace）+ Sphere Overlap Bounce**：地面锁定使用 Line Trace 向下射线检测，碰撞反弹使用 Sphere Overlap（1m 半径检测 + 1.2m 反弹距离）
- **无人机注册为 CARLA Actor**：无人机以 `airsim.drone` 类型注册到 CARLA 的 `ActorDispatcher`，可通过 `world.get_actors()` 查询
- **CARLA ↔ AirSim 坐标精确对齐**：校准误差为 0.0000m
- **CarlaAir.sh 启动脚本改进**：Vulkan 自动检测（支持 NVIDIA/AMD/Intel GPU），默认 Epic 画质，OpenGL fallback 兜底

### 4.3 v0.1.3 详细变更

- **FPSYaw_ 初始化修复**：之前 FPSYaw_ 变量未正确初始化，导致按 W 键时无人机飞行方向不正确
- **默认第三人称视角**：移除了 V 键视角切换功能，默认使用第三人称视角

### 4.4 v0.1.4 详细变更

- **减号键黑屏修复**：减号键（-）与 UE4 内部快捷键冲突导致黑屏，速度调节键改为 ] 和 [
- **移除 M 键地图切换**：避免误操作
- **碰撞检测增强**：1m Sphere 检测 + 1.2m Bounce 反弹 + AirSim API 层面碰撞追踪（三重保障）
- **ROS2 适配验证**：确认 63 个 ROS 话题正常（43 CARLA + 14 AirSim + 6 通用），rviz2 可视化正常
- **第一次正式发布包**：
  - 二进制包：`CarlaAir-v0.1.4/`（18GB）
  - 源代码包：`CarlaAir-v0.1.4-source/`（653MB）
- **5 个轨迹录制/回放脚本**：
  - `record_walker.py` — 行人轨迹录制
  - `record_vehicle.py` — 车辆轨迹录制
  - `record_drone.py` — 无人机轨迹录制
  - `replay_trajectories.py` — 轨迹回放
  - `record_video.py` — 视频录制
- **setup.py + requirements.txt**：支持 `pip install -e .` 安装
- **README.md**：完整使用教程

### 4.5 v0.1.5 详细变更（当前版本）

- **碰撞系统重写**：12-direction LineTrace（8 个水平方向 + 4 个对角向下方向），替代之前的 Sphere Overlap 方案，精度和性能均更优
- **鼠标滚轮速度调节**：替代之前的 ]/[ 按键方案，操作更直觉
- **P 键碰撞模式切换**：物理碰撞模式 ↔ 无敌穿透模式
- **H 键双语帮助菜单**：纯黑背景（α=0.92），大号粗体字，中英文双语
- **发布包**：
  - 二进制包：`CarlaAir-v0.1.5/`（18GB），路径：`/mnt/data1/tianle/CarlaAir-v0.1.5/`
  - 源代码包：651MB

---

## 五、关键技术成就与 Bug 修复

### 5.1 核心技术成就

| 成就 | 说明 |
|------|------|
| 单进程空地联合仿真 | CARLA + AirSim 共享同一 UE4 进程，无需跨进程通信 |
| 统一 GameMode 架构 | `ASimWorldGameMode` 统一控制器，CARLA 为主控，AirSim 为附属组件 |
| 坐标精确对齐 | CARLA ↔ AirSim 坐标转换误差 0.0000m |
| 12-direction LineTrace 碰撞 | 自研碰撞检测系统，解决了 UE4 Sweep 在 CARLA 地图中的 false collision 问题 |
| 内置 FPS 控制 | C++ 原生实现的无人机操控，无需 Python |
| ROS2 双 Bridge 63 话题 | CARLA ROS Bridge + AirSim ROS Bridge 同时运行 |
| 传感器 6 路并发稳定 | 3cam + LiDAR + IMU + GNSS 10 分钟无崩溃 |
| 89/89 API 测试全通过 | 完整 API 兼容性验证 |

### 5.2 已修复的关键 Bug

| Bug | 原因 | 修复方案 | 版本 |
|-----|------|----------|------|
| 无人机无法飞行（false sweep collision） | CARLA 地图中存在大量虚假碰撞体 | Sweep→Teleport 移动方式 | v0.1.2 |
| W 键方向不正确 | FPSYaw_ 变量未初始化 | 初始化修复 | v0.1.3 |
| 减号键黑屏 | 与 UE4 内部快捷键冲突 | 速度键改为 ]/[（后改为鼠标滚轮） | v0.1.4 / v0.1.5 |
| 车辆穿越地面 | `SetNewWorldOrigin` 导致坐标偏移 | 禁用 `SetNewWorldOrigin` | v0.1.2 |
| 传感器并发崩溃 | IMU `GetOwner` 返回 null + GPU readback 过载 | null guard + GPU readback throttle | v0.1.4 |
| CARLA sync mode + AirSim simGetImages 渲染死锁 | 两个系统争抢渲染资源 | 所有相机切换为 CARLA 传感器 | Demo 录制期间 |
| Walker AI 不工作 | AI Controller 未正确绑定 | 修复 AI Controller 绑定 | 第二轮测试 |

---

## 六、测试结果汇总

### 6.1 API 测试

- **结果**：89/89 PASS（100%）
- 覆盖 CARLA API 和 AirSim API 的所有关键接口

### 6.2 第一轮正式测试

- **测试用例数**：48
- **通过率**：97.8%（47/48 通过，1 个边界情况）

### 6.3 第二轮专家测试

| 测试项 | 结果 |
|--------|------|
| ROS 话题验证 | 39 个话题数据正确 |
| Walker AI | 修复后正常工作 |
| 传感器边界 | 确认边界条件 |

### 6.4 传感器并发稳定性测试

- **配置**：6 个传感器同时运行（3 Camera + LiDAR + IMU + GNSS）
- **时长**：10 分钟连续运行
- **结果**：稳定，无崩溃
- **帧数**：Camera 组 28,986 帧 / LiDAR+IMU 24,866 帧 / GNSS 59,053 帧

### 6.5 综合稳定性测试

- 30 辆地面车辆 + 1 架 AirSim 无人机同时运行，稳定
- ROS2 双 Bridge 63 话题同时发布，稳定

---

## 七、示例脚本（17 个）

### 7.1 原始 12 个示例脚本

覆盖 CARLA、AirSim、联合使用三类场景的 Python 示例脚本（具体列表见代码仓库 README）。

### 7.2 新增 5 个轨迹录制/回放脚本（v0.1.4 新增）

| 脚本 | 功能 |
|------|------|
| `record_walker.py` | 录制行人运动轨迹 |
| `record_vehicle.py` | 录制车辆运动轨迹 |
| `record_drone.py` | 录制无人机飞行轨迹 |
| `replay_trajectories.py` | 回放已录制的轨迹（支持行人/车辆/无人机） |
| `record_video.py` | 录制多相机视频 |

---

## 八、Demo 录制（已完成）

### 8.1 概况

| 指标 | 数值 |
|------|------|
| 已录制 Demo 数量 | **30 个** |
| 使用地图 | 7 张（Town01-Town05、Town10HD） |
| 总数据量 | 2.1 GB |
| 分辨率 | 1920×1080 |

### 8.2 标准画面布局

所有 Demo 使用统一的 **4-grid 分屏布局**：

| 位置 | 相机 | 说明 |
|------|------|------|
| 左上 | CAM-A | Drone FPV（无人机第一人称视角） |
| 右上 | CAM-B | Drone 3rd Person（无人机第三人称视角） |
| 左下 | CAM-C | Vehicle FPV（车辆第一人称视角） |
| 右下 | CAM-D | Vehicle 3rd Person（车辆第三人称视角） |

### 8.3 关键技术修复

录制过程中发现并修复了一个严重问题：**CARLA sync mode + AirSim `simGetImages` 渲染死锁**。原因是两个系统争抢渲染资源，解决方案是将所有相机切换为 CARLA 传感器实现。

---

## 九、发布包信息

### 9.1 二进制包

| 项目 | 内容 |
|------|------|
| 包名 | `CarlaAir-v0.1.5/` |
| 大小 | 18 GB |
| 路径 | `/mnt/data1/tianle/CarlaAir-v0.1.5/` |
| 内容 | 预编译二进制文件，包含 13 张地图，一行命令启动 |
| 启动脚本 | `CarlaAir.sh`（Vulkan 自动检测，支持 NVIDIA/AMD/Intel，默认 Epic 画质，OpenGL fallback） |

### 9.2 源代码包

| 项目 | 内容 |
|------|------|
| 大小 | 651 MB |
| 内容 | 完整 UE4 项目源码 |
| 安装方式 | `pip install -e .`（通过 setup.py + requirements.txt） |

### 9.3 文档

- README.md：完整使用教程
- 17 个 Python 示例脚本（12 原始 + 5 录制脚本）

---

## 十、发布计划与当前状态

| 交付物 | 状态 | 说明 |
|--------|------|------|
| 预编译二进制包（18GB） | **已完成** | v0.1.5，路径 `/mnt/data1/tianle/CarlaAir-v0.1.5/` |
| 源代码包（651MB） | **已完成** | 含 setup.py、requirements.txt、README.md |
| Python 示例脚本（17个） | **已完成** | 12 原始 + 5 轨迹录制脚本 |
| Demo 视频（30个） | **已完成** | 7 张地图，2.1GB，1920×1080 |
| API 测试 | **已完成** | 89/89 PASS |
| 系统测试（两轮） | **已完成** | 第一轮 97.8%，第二轮 ROS + Walker AI + 传感器边界 |
| **技术报告（arXiv paper）** | **即将开始最终撰写** | 这也是本文档的直接目的 |
| 项目网页 | 待开发 | — |
| GitHub Release | 待发布 | — |

### 发布渠道

- **arXiv**：技术报告
- **GitHub**：代码、二进制包、Issue 追踪
- **Hugging Face**：模型/数据集页面（如适用）
- **项目网页**：独立网站，Demo 和文档
- **社区推广**：X（Twitter）、Reddit、知乎

---

## 十一、已验证的典型应用场景

1. **无人机追踪地面车辆**：无人机实时跟随地面车辆飞行，同时采集空中和地面两个视角的同步数据
2. **航拍交通监视**：无人机悬停在十字路口上方，俯拍下方自动驾驶车辆的交通流
3. **多模态数据集采集**：同时采集 18 路传感器数据（地面 + 空中），帧号一致，用于训练 AI 模型
4. **天气一致性仿真**：天气变化（晴→雨→暴雨）同时影响地面和空中所有传感器，物理上完全一致
5. **行人环境中的无人机导航**：无人机在有行人的城市街道低空飞行

---

## 十二、一句话总结

CarlaAir 是第一个把 CARLA 和 AirSim 合并进同一个 UE4 进程的开源空地联合仿真平台，支持 18 路传感器同步采集、内置 FPS 无人机控制、12-direction LineTrace 碰撞物理、ROS2 双 Bridge（63 话题）、CARLA↔AirSim 坐标精确对齐（0.0000m 误差），89/89 API 测试全通过，已完成两轮系统测试（97.8% 通过率）和 30 个 Demo 录制，技术报告即将进入最终撰写阶段。

---

*文档版本：v2.0 | 日期：2026-03-18 | 用途：技术报告撰写 AI Agent 背景输入*
