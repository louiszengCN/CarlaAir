# CarlaAir 从源码编译指南

## 前提条件

### 系统要求
- **操作系统**: Ubuntu 18.04 / 20.04 / 22.04 (推荐 22.04)
- **GPU**: NVIDIA GPU，显存 >= 8GB (推荐 12GB+)
- **内存**: >= 32GB RAM
- **磁盘**: >= 100GB 可用空间
- **显示**: 需要 X11 显示服务 (或 VNC/Xvfb)

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

## 源码位置

| 组件 | 路径 |
|------|------|
| UE4 4.26 (CARLA fork) | `/mnt/data1/tianle/carla_ue4/` |
| CARLA 源码 | `/mnt/data1/tianle/carla_source/` |
| AirSim 插件 | `Unreal/CarlaUE4/Plugins/AirSim/` |
| CarlaAir 核心文件 | `Plugins/AirSim/Source/SimWorldGameMode.h/.cpp` |

## 编译步骤

### 1. 设置环境变量

```bash
export UE4_ROOT=/mnt/data1/tianle/carla_ue4
cd /mnt/data1/tianle/carla_source
```

### 2. 编译 AirSim 模块（~100 秒）

```bash
# 仅编译 AirSim 插件（快速迭代时使用）
make CarlaUE4Editor ARGS="-module=AirSim"
# 或直接使用 Build.sh
./Util/BuildTools/BuildCarlaUE4.sh --build
```

### 3. 编译 CARLA 模块（~290 秒）

```bash
# 编译 CARLA 插件
make CarlaUE4Editor ARGS="-module=Carla"
```

### 4. 在 Editor 模式中运行（开发调试）

```bash
# 使用一键启动脚本
./carlaAir.sh                    # 默认 Town10HD, 1280x720
./carlaAir.sh Town03             # 指定地图
./carlaAir.sh Town10HD 1920 1080 # 指定分辨率
```

### 5. 打包 Shipping 构建（发布用）

```bash
# 完整打包流程（编译 ~805s + Cook ~2h）
./Util/BuildTools/Package.sh --config=Shipping --no-zip

# 输出目录
ls Dist/CARLA_Shipping_*/LinuxNoEditor/
```

**注意**: Package.sh 在最后 rsync PythonAPI wheel 时可能失败（因为 `.whl` 文件未构建），但这不影响主要构建。手动复制 PythonAPI 文件到输出目录即可。

### 6. 运行 Shipping 构建

```bash
cd Dist/CARLA_Shipping_*/LinuxNoEditor/
./CarlaUE4.sh /Game/Carla/Maps/Town10HD -nosound -carla-rpc-port=2000
```

## 关键配置文件

### DefaultGame.ini

位于 `Unreal/CarlaUE4/Config/DefaultGame.ini`。打包时需要确保以下条目存在：

```ini
# AirSim 资源目录（必须）
+DirectoriesToAlwaysCook=(Path="/AirSim/Blueprints")
+DirectoriesToAlwaysCook=(Path="/AirSim/HUDAssets")
+DirectoriesToAlwaysCook=(Path="/AirSim/Models")
+DirectoriesToAlwaysCook=(Path="/AirSim/Weather")
+DirectoriesToAlwaysCook=(Path="/AirSim/StarterContent")
+DirectoriesToAlwaysCook=(Path="/AirSim/VehicleAdv")

# AirSim 地图
+MapsToCook=(FilePath="/AirSim/AirSimAssets")
```

### settings.json

位于 `~/Documents/AirSim/settings.json`，控制 AirSim 行为：

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "SimpleFlight": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true
        }
    }
}
```

## 编译时间参考

| 步骤 | 时间 | 说明 |
|------|------|------|
| AirSim 模块编译 | ~100s | 仅 AirSim 相关 C++ |
| CARLA 模块编译 | ~290s | CARLA 插件 C++ |
| Shipping 编译 | ~805s | 656 个编译单元 |
| Cook (资源烘焙) | ~2h | 13459 个包 |
| TaggedMaterials | ~977s | 实例分割材质 |
| **总计 (完整打包)** | **~3.5h** | 首次；增量编译更快 |

## 常见问题

### Q: 首次启动非常慢（~10分钟）
**A**: 首次运行时 UE4 需要编译 DDC (Derived Data Cache) 和着色器。后续启动约 1 分钟。

### Q: Package.sh 最后报错
**A**: 通常是 PythonAPI wheel rsync 失败（`set -e` 导致脚本中断）。主要的可执行文件和资源已经正确打包，手动复制 PythonAPI 即可。

### Q: 打包后的构建缺少地图
**A**: 检查 `DefaultGame.ini` 中的 `MapsToCook` 条目。确保包含所有需要的地图。

### Q: AirSim API 无法连接
**A**: 检查 `~/Documents/AirSim/settings.json` 是否存在且格式正确。Editor 模式下需要等待 AirSim 端口 41451 就绪。
