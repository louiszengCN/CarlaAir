# CarlaAir 从源码编译指南

> **注意**：对于绝大多数学术研究用户，我们强烈建议直接下载**预编译的二进制发行版**（见主页 README）。源码编译过程极为复杂，且严重依赖特定的系统环境和硬件配置，主要面向需要深度定制底层引擎架构的资深开发者。

## 前提条件

### 系统要求
- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04
- **GPU**: NVIDIA GPU，显存 >= 8GB
- **内存**: >= 32GB RAM
- **磁盘**: >= 100GB 可用空间

### 软件依赖
```bash
# 基础编译工具
sudo apt install build-essential cmake git clang-10 lld-10

# UE4 依赖
sudo apt install libvulkan1 vulkan-utils mesa-vulkan-drivers

# Python 环境
conda create -n simworld python=3.7
conda activate simworld
pip install carla==0.9.16 airsim numpy opencv-python pygame
```

## 源码获取

CarlaAir 的构建依赖于高度定制化的 Unreal Engine 4.26 分支和特定的 CARLA 版本。

1. 获取 Epic Games 账号并关联 GitHub，以获得 UE4 源码访问权限。
2. 获取 CARLA 0.9.16 (ue4-dev 分支) 的源码。
3. 将本仓库中的 CarlaAir 核心代码（位于 `core_source`）合并到 CARLA 源码的对应位置。

> **环境配置提示**：您需要自行配置 `$UE4_ROOT` 和 `$CARLA_ROOT` 环境变量，确保它们指向正确的源码目录。

## 编译步骤

### 1. 编译引擎与基础模块

在开始编译 CarlaAir 之前，请确保您已经成功编译了 Unreal Engine 4.26 和基础的 CARLA 模块。

```bash
export UE4_ROOT=<your_ue4_root>
cd <your_carla_root>

# 编译基础引擎（此步骤可能需要数小时）
make UnrealEngine
```

### 2. 编译 AirSim 插件

CarlaAir 深度集成了 AirSim。您需要首先编译 AirSim 模块：

```bash
# 编译 AirSim 插件
make CarlaUE4Editor ARGS="-module=AirSim"
```

### 3. 编译 CarlaAir 核心

在 AirSim 模块编译完成后，编译融合了空地协同逻辑的 CARLA 模块：

```bash
# 编译 CARLA 插件
make CarlaUE4Editor ARGS="-module=Carla"
```

### 4. Editor 模式调试

编译完成后，您可以在 UE4 Editor 中启动 CarlaAir 进行调试：

```bash
make launch
```

### 5. 打包 Shipping 构建

当您完成所有定制开发后，可以打包为独立的二进制发行版：

```bash
# 完整打包流程（编译 + Cook，通常需要 3-4 小时）
./Util/BuildTools/Package.sh --config=Shipping --no-zip
```

构建产物将输出到 `Dist/CARLA_Shipping_*/LinuxNoEditor/` 目录下。

## 常见问题

### Q: 编译时提示找不到 `features.h` 或其他头文件？
**A**: 这是由于系统头文件路径未正确映射到 UE4 的构建系统中。请检查您的 clang 工具链配置，或尝试使用 `Build.sh` 脚本替代 `make` 命令。

### Q: 启动 Editor 时发生崩溃？
**A**: 请确保您的显卡驱动支持 Vulkan，并且已正确安装 `vulkan-utils`。此外，首次启动时需要编译大量着色器，请耐心等待。

### Q: 打包后的版本中没有无人机？
**A**: 请检查 `Unreal/CarlaUE4/Config/DefaultGame.ini`，确保 AirSim 的蓝图和资源目录已正确添加到 `DirectoriesToAlwaysCook` 列表中。
