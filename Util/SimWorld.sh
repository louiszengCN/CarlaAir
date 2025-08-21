#!/bin/bash
# SimWorld.sh — Launch packaged CARLA + AirSim simulator
# This script is for the PACKAGED (Shipping) build, not the Editor.
#
# Usage:
#   ./SimWorld.sh                    # Default: Town10HD, windowed 1280x720
#   ./SimWorld.sh Town03             # Use a different map
#   ./SimWorld.sh Town10HD 1920 1080 # Custom resolution
#   ./SimWorld.sh --help             # Show help

set -e

# ========================== Configuration ==========================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="${SCRIPT_DIR}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
LOGFILE="/tmp/simworld.log"

DEFAULT_MAP="Town10HD"
DEFAULT_RESX=1280
DEFAULT_RESY=720
CARLA_PORT=2000
AIRSIM_PORT=41451
QUALITY="Low"

# ========================== Help ==========================
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<'EOF'
SimWorld.sh — Packaged CARLA + AirSim Simulator Launcher

Usage: ./SimWorld.sh [MAP] [ResX] [ResY] [OPTIONS]

Arguments:
  MAP          Map name (default: Town10HD)
               Available: Town01-05, Town10HD (and _Opt variants), OpenDriveMap, EmptyMap
  ResX ResY    Window resolution (default: 1280 720)

Options:
  --help, -h       Show this help
  --fg             Run in foreground (default: background)
  --quality=LEVEL  Quality: Low/Medium/High/Epic (default: Low)
  --port=PORT      CARLA RPC port (default: 2000)
  --kill           Kill running instance and exit
  --log            Tail the log file

After launch:
  CARLA API:   python -c "import carla; c=carla.Client('localhost',2000)"
  AirSim API:  python -c "import airsim; c=airsim.MultirotorClient(port=41451)"

AirSim settings: ~/Documents/AirSim/settings.json
  If you don't have one, copy the example:
    mkdir -p ~/Documents/AirSim
    cp AirSimConfig/settings.json.example ~/Documents/AirSim/settings.json
EOF
    exit 0
fi

# ========================== Kill mode ==========================
if [[ "$1" == "--kill" ]]; then
    echo "Stopping SimWorld..."
    pkill -f "CarlaUE4-Linux-Shipping" 2>/dev/null && echo "Stopped." || echo "Not running."
    exit 0
fi

# ========================== Log mode ==========================
if [[ "$1" == "--log" ]]; then
    exec tail -f "$LOGFILE"
fi

# ========================== Parse args ==========================
MAP="$DEFAULT_MAP"
RESX="$DEFAULT_RESX"
RESY="$DEFAULT_RESY"
FOREGROUND=false

for arg in "$@"; do
    case "$arg" in
        --fg) FOREGROUND=true ;;
        --quality=*) QUALITY="${arg#*=}" ;;
        --port=*) CARLA_PORT="${arg#*=}" ;;
        Town*|OpenDrive*|Empty*) MAP="$arg" ;;
        [0-9][0-9][0-9]*)
            if [[ "$RESX" == "$DEFAULT_RESX" ]]; then
                RESX="$arg"
            else
                RESY="$arg"
            fi
            ;;
    esac
done

# ========================== Preflight checks ==========================
if ss -tlnp 2>/dev/null | grep -q ":${CARLA_PORT} "; then
    echo "WARNING: Port $CARLA_PORT is already in use!"
    echo "  Use './SimWorld.sh --kill' to stop it first."
    exit 1
fi

if [[ ! -f "$BINARY" ]]; then
    echo "ERROR: Shipping binary not found at $BINARY"
    echo "  Make sure you're running this from the packaged build directory."
    exit 1
fi

if [[ -z "$DISPLAY" ]]; then
    export DISPLAY=:1
fi

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

# Check AirSim settings
if [[ ! -f "$HOME/Documents/AirSim/settings.json" ]]; then
    echo "WARNING: AirSim settings not found at ~/Documents/AirSim/settings.json"
    if [[ -f "${SCRIPT_DIR}/AirSimConfig/settings.json.example" ]]; then
        echo "  Copy the example: mkdir -p ~/Documents/AirSim && cp ${SCRIPT_DIR}/AirSimConfig/settings.json.example ~/Documents/AirSim/settings.json"
    fi
    echo ""
fi

# ========================== Launch ==========================
MAP_PATH="/Game/Carla/Maps/${MAP}"

CMD=(
    "$BINARY"
    "$MAP_PATH"
    -windowed
    -ResX="$RESX"
    -ResY="$RESY"
    -carla-rpc-port="$CARLA_PORT"
    -quality-level="$QUALITY"
    -unattended
    -nosound
)

echo "============================================"
echo "  SimWorld — CARLA + AirSim (Packaged)"
echo "============================================"
echo "  Map:        $MAP"
echo "  Resolution: ${RESX}x${RESY}"
echo "  Quality:    $QUALITY"
echo "  CARLA port: $CARLA_PORT"
echo "  AirSim port: $AIRSIM_PORT"
echo "  Log:        $LOGFILE"
echo "============================================"

if $FOREGROUND; then
    echo "Starting in foreground..."
    "${CMD[@]}" 2>&1 | tee "$LOGFILE"
else
    echo "Starting in background..."
    nohup "${CMD[@]}" > "$LOGFILE" 2>&1 &
    PID=$!
    echo "  PID: $PID"
    echo ""
    echo "Waiting for servers to start..."

    MAX_WAIT=300
    ELAPSED=0
    CARLA_READY=false
    AIRSIM_READY=false

    while [[ $ELAPSED -lt $MAX_WAIT ]]; do
        if ! kill -0 $PID 2>/dev/null; then
            echo "ERROR: SimWorld crashed during startup!"
            echo "Check log: tail -50 $LOGFILE"
            exit 1
        fi

        if ! $CARLA_READY && ss -tlnp 2>/dev/null | grep -q ":${CARLA_PORT} " ; then
            CARLA_READY=true
            echo "  [$(date +%H:%M:%S)] CARLA server ready (port $CARLA_PORT)"
        fi
        if ! $AIRSIM_READY && ss -tlnp 2>/dev/null | grep -q ":${AIRSIM_PORT} " ; then
            AIRSIM_READY=true
            echo "  [$(date +%H:%M:%S)] AirSim server ready (port $AIRSIM_PORT)"
        fi

        if $CARLA_READY && $AIRSIM_READY; then
            echo ""
            echo "============================================"
            echo "  Ready! Both servers are up."
            echo "============================================"
            echo ""
            echo "Quick test:"
            echo "  python3 -c \"import carla; c=carla.Client('localhost',$CARLA_PORT); print('CARLA:', c.get_world().get_map().name)\""
            echo "  python3 -c \"import airsim; c=airsim.MultirotorClient(port=$AIRSIM_PORT); c.confirmConnection()\""
            echo ""
            echo "Stop:  ./SimWorld.sh --kill"
            echo "Log:   ./SimWorld.sh --log"
            exit 0
        fi

        sleep 5
        ELAPSED=$((ELAPSED + 5))

        if [[ $((ELAPSED % 30)) -eq 0 ]]; then
            LAST_LOG=$(tail -1 "$LOGFILE" 2>/dev/null | head -c 120)
            echo "  [$(date +%H:%M:%S)] Waiting... (${ELAPSED}s) $LAST_LOG"
        fi
    done

    echo "WARNING: Timed out after ${MAX_WAIT}s. Servers may still be starting."
    echo "Check: ss -tlnp | grep -E '$CARLA_PORT|$AIRSIM_PORT'"
    echo "Log:   tail -f $LOGFILE"
fi
