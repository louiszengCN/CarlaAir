# CarlaAir 安装与运行指南

CarlaAir (SimWorld) 是一个空地一体联合仿真平台，将 CARLA 地面仿真与 AirSim 无人机飞行仿真统一在同一个虚拟世界中。

## 系统要求

| 项目 | 最低要求 | 推荐配置 |
|------|---------|---------|
| 操作系统 | Ubuntu 18.04 | Ubuntu 22.04 |
| GPU | NVIDIA GTX 1080 (8GB) | NVIDIA RTX 3080 (10GB+) |
| GPU 驱动 | 470+ (Vulkan 支持) | 535+ |
| 内存 | 16 GB | 32 GB |
| 磁盘 | 25 GB | 50 GB |
| 显示 | X11 或 VNC | 本地显示器 |
| Python | 3.7+ | 3.8 |

## 快速安装（3 步）

### 第 1 步：安装 Python 依赖

```bash
# 使用 conda（推荐）
conda create -n simworld python=3.8 -y
conda activate simworld
pip install carla==0.9.16 airsim numpy opencv-python pygame

# 或使用 pip
pip install carla==0.9.16 airsim numpy opencv-python pygame
```

### 第 2 步：配置 AirSim 设置

```bash
# 创建 AirSim 设置目录
mkdir -p ~/Documents/AirSim

# 复制默认设置文件
cp settings.json ~/Documents/AirSim/settings.json
```

settings.json 内容（多旋翼无人机模式）:
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "SimpleFlight": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true,
            "Cameras": {
                "0": {
                    "CaptureSettings": [
                        { "ImageType": 0, "Width": 1280, "Height": 960 }
                    ],
                    "X": 0.5, "Y": 0.0, "Z": 0.1,
                    "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0
                }
            }
        }
    }
}
```

### 第 3 步：启动仿真器

```bash
# 进入安装目录（Shipping 构建）
cd <path_to_carlaair>/LinuxNoEditor   # 解压后的 CarlaAir 发行包目录

# 一键启动（默认 Town10HD 地图）
./carla_air.sh

# 或手动启动
./CarlaUE4.sh /Game/Carla/Maps/Town10HD -nosound -carla-rpc-port=2000 -windowed -ResX=1280 -ResY=720
```

## 启动脚本：carla_air.sh

将下面的 `carla_air.sh` 放到安装目录中即可一键启动：

```bash
#!/bin/bash
# carla_air.sh — CarlaAir 一键启动脚本（Shipping 构建版）
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MAP="${1:-Town10HD}"
RESX="${2:-1280}"
RESY="${3:-720}"
PORT=2000
AIRSIM_PORT=41451

# 检查 AirSim 设置
if [ ! -f ~/Documents/AirSim/settings.json ]; then
    echo "WARNING: ~/Documents/AirSim/settings.json not found!"
    echo "Creating default settings..."
    mkdir -p ~/Documents/AirSim
    cat > ~/Documents/AirSim/settings.json << 'EOF'
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
EOF
fi

# Vulkan ICD
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
[ -z "$DISPLAY" ] && export DISPLAY=:1

echo "============================================"
echo "  CarlaAir - Air-Ground Co-Simulation"
echo "============================================"
echo "  Map:        $MAP"
echo "  Resolution: ${RESX}x${RESY}"
echo "  CARLA port: $PORT"
echo "  AirSim port: $AIRSIM_PORT"
echo "============================================"

# 启动
"$SCRIPT_DIR/CarlaUE4.sh" \
    /Game/Carla/Maps/$MAP \
    -nosound \
    -carla-rpc-port=$PORT \
    -windowed \
    -ResX=$RESX \
    -ResY=$RESY \
    -quality-level=Low &

PID=$!
echo "PID: $PID"
echo "Waiting for servers..."

# 等待两个端口就绪
for i in $(seq 1 120); do
    CARLA_OK=false; AIRSIM_OK=false
    ss -tlnp 2>/dev/null | grep -q ":${PORT} " && CARLA_OK=true
    ss -tlnp 2>/dev/null | grep -q ":${AIRSIM_PORT} " && AIRSIM_OK=true
    if $CARLA_OK && $AIRSIM_OK; then
        echo ""
        echo "Ready! CARLA(:$PORT) + AirSim(:$AIRSIM_PORT) both running."
        echo ""
        echo "Test commands:"
        echo "  python3 -c \"import carla; c=carla.Client('localhost',$PORT); print(c.get_world().get_map().name)\""
        echo "  python3 -c \"import airsim; c=airsim.MultirotorClient(port=$AIRSIM_PORT); c.confirmConnection()\""
        exit 0
    fi
    if ! kill -0 $PID 2>/dev/null; then
        echo "ERROR: Process crashed during startup!"
        exit 1
    fi
    [ $((i % 6)) -eq 0 ] && echo "  Still waiting... (${i}*5s)"
    sleep 5
done
echo "WARNING: Timeout. Servers may still be starting."
```

## 验证安装

启动后，打开新终端：

```bash
conda activate simworld

# 测试 CARLA 连接
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10)
world = client.get_world()
print(f'CARLA OK: {world.get_map().name}')
print(f'Spawn points: {len(world.get_map().get_spawn_points())}')
"

# 测试 AirSim 连接
python3 -c "
import airsim
client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
print('AirSim OK: Connected')
"
```

## 可用地图

| 地图名 | 描述 | 推荐用途 |
|--------|------|---------|
| Town10HD | 大型沿海城市 (默认) | 综合演示、交通仿真 |
| Town01 | 小型城镇 | 基础测试 |
| Town02 | 小型住宅区 | 行人场景 |
| Town03 | 城市+隧道 | 复杂道路 |
| Town04 | 高速公路+城市 | 高速场景 |
| Town05 | 多车道城市 | 交通管理 |

切换地图: `./carla_air.sh Town03`

## 停止仿真器

```bash
# 方法 1: 关闭窗口
# 方法 2: 命令行
pkill -f "CarlaUE4-Linux-Shipping"
```

## 目录结构

```
LinuxNoEditor/
├── CarlaUE4.sh              # 原始 UE4 启动脚本
├── carla_air.sh             # CarlaAir 一键启动脚本（需要手动创建）
├── CarlaUE4/
│   ├── Binaries/Linux/      # 可执行文件
│   └── Content/             # 地图和资源
├── Engine/                   # UE4 运行时
└── PythonAPI/               # Python 客户端库
    └── carla/
        └── dist/            # carla-0.9.16 wheel
```

## 常见问题

### Q: 启动后黑屏或无画面
**A**: 检查 GPU 驱动和 Vulkan 支持：`vulkaninfo | head -5`

### Q: CARLA 端口连接超时
**A**: 首次启动需要 2-5 分钟加载资源。用 `ss -tlnp | grep 2000` 检查端口。

### Q: AirSim 无法连接
**A**: 确认 `~/Documents/AirSim/settings.json` 存在。端口 41451 需要在 CARLA 之后才就绪。

### Q: 分辨率太低/画质差
**A**: 修改启动参数：`./carla_air.sh Town10HD 1920 1080`。编辑脚本中的 `quality-level` 为 `Medium` 或 `High`。

### Q: 内存不足
**A**: Town10HD 需要约 8GB GPU 显存。可尝试 `-quality-level=Low` 或使用更小的地图如 Town01。
