#!/bin/bash
# carla_air.sh — CarlaAir 一键启动脚本（Shipping 构建版）
# 用法:
#   ./carla_air.sh                    # 默认: Town10HD, 1280x720
#   ./carla_air.sh Town03             # 指定地图
#   ./carla_air.sh Town10HD 1920 1080 # 指定分辨率
#   ./carla_air.sh --kill             # 停止运行中的实例
#   ./carla_air.sh --help             # 帮助

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MAP="${1:-Town10HD}"
RESX="${2:-1280}"
RESY="${3:-720}"
PORT=2000
AIRSIM_PORT=41451
QUALITY="Low"

# === Help ===
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<'EOF'
CarlaAir — Air-Ground Co-Simulation Platform

Usage: ./carla_air.sh [MAP] [ResX] [ResY]

Arguments:
  MAP          Map name (default: Town10HD)
               Available: Town01-05, Town10HD
  ResX ResY    Window resolution (default: 1280 720)

Options:
  --help       Show this help
  --kill       Stop running instance
  --status     Check server status

After launch:
  CARLA API:   python -c "import carla; c=carla.Client('localhost',2000)"
  AirSim API:  python -c "import airsim; c=airsim.MultirotorClient(port=41451)"

AirSim settings: ~/Documents/AirSim/settings.json
EOF
    exit 0
fi

# === Kill ===
if [[ "$1" == "--kill" ]]; then
    echo "Stopping CarlaAir..."
    pkill -f "CarlaUE4-Linux-Shipping" 2>/dev/null && echo "Stopped." || echo "Not running."
    exit 0
fi

# === Status ===
if [[ "$1" == "--status" ]]; then
    echo "Port $PORT (CARLA):  $(ss -tlnp 2>/dev/null | grep -q ":${PORT} " && echo 'RUNNING' || echo 'NOT RUNNING')"
    echo "Port $AIRSIM_PORT (AirSim): $(ss -tlnp 2>/dev/null | grep -q ":${AIRSIM_PORT} " && echo 'RUNNING' || echo 'NOT RUNNING')"
    exit 0
fi

# === Check if already running ===
if ss -tlnp 2>/dev/null | grep -q ":${PORT} "; then
    echo "WARNING: Port $PORT already in use! CarlaAir may already be running."
    echo "  Use './carla_air.sh --kill' to stop it first."
    exit 1
fi

# === Ensure AirSim settings exist ===
if [ ! -f ~/Documents/AirSim/settings.json ]; then
    echo "Creating default AirSim settings..."
    mkdir -p ~/Documents/AirSim
    cat > ~/Documents/AirSim/settings.json << 'SETTINGS'
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
SETTINGS
fi

# === Vulkan / Display ===
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
[ -z "$DISPLAY" ] && export DISPLAY=:1

# === Launch ===
echo "============================================"
echo "  CarlaAir - Air-Ground Co-Simulation"
echo "============================================"
echo "  Map:        $MAP"
echo "  Resolution: ${RESX}x${RESY}"
echo "  Quality:    $QUALITY"
echo "  CARLA port: $PORT"
echo "  AirSim port: $AIRSIM_PORT"
echo "============================================"
echo ""

"$SCRIPT_DIR/CarlaUE4.sh" \
    /Game/Carla/Maps/$MAP \
    -nosound \
    -carla-rpc-port=$PORT \
    -windowed \
    -ResX=$RESX \
    -ResY=$RESY \
    -quality-level=$QUALITY &

PID=$!
echo "PID: $PID"
echo "Waiting for servers to start..."

# === Wait for both ports ===
MAX_WAIT=600
ELAPSED=0
CARLA_READY=false
AIRSIM_READY=false

while [ $ELAPSED -lt $MAX_WAIT ]; do
    # Check if process crashed
    if ! kill -0 $PID 2>/dev/null; then
        echo "ERROR: CarlaAir process crashed during startup!"
        exit 1
    fi

    # Check CARLA port
    if ! $CARLA_READY && ss -tlnp 2>/dev/null | grep -q ":${PORT} "; then
        CARLA_READY=true
        echo "  [$(date +%H:%M:%S)] CARLA server ready (port $PORT)"
    fi

    # Check AirSim port
    if ! $AIRSIM_READY && ss -tlnp 2>/dev/null | grep -q ":${AIRSIM_PORT} "; then
        AIRSIM_READY=true
        echo "  [$(date +%H:%M:%S)] AirSim server ready (port $AIRSIM_PORT)"
    fi

    if $CARLA_READY && $AIRSIM_READY; then
        echo ""
        echo "============================================"
        echo "  Ready! Both servers are running."
        echo "============================================"
        echo ""
        echo "Quick test:"
        echo "  python3 -c \"import carla; c=carla.Client('localhost',$PORT); print('CARLA:', c.get_world().get_map().name)\""
        echo "  python3 -c \"import airsim; c=airsim.MultirotorClient(port=$AIRSIM_PORT); c.confirmConnection(); print('AirSim: OK')\""
        echo ""
        echo "Stop: ./carla_air.sh --kill"
        exit 0
    fi

    sleep 5
    ELAPSED=$((ELAPSED + 5))

    if [ $((ELAPSED % 30)) -eq 0 ]; then
        echo "  [$(date +%H:%M:%S)] Still waiting... (${ELAPSED}s)"
    fi
done

echo "WARNING: Timed out after ${MAX_WAIT}s. Servers may still be starting."
echo "Check: ./carla_air.sh --status"
