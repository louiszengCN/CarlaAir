#!/bin/bash
# carlaAir.sh — Launch unified CARLA + AirSim simulator
# Usage:
#   ./carlaAir.sh                    # Default: Town10HD, windowed 1280x720
#   ./carlaAir.sh Town03             # Use a different map
#   ./carlaAir.sh Town10HD 1920 1080 # Custom resolution
#   ./carlaAir.sh --help             # Show help

set -e

# ========================== Configuration ==========================
UE4_ROOT="/mnt/data1/tianle/carla_ue4"
PROJECT="/mnt/data1/tianle/carla_source/Unreal/CarlaUE4/CarlaUE4.uproject"
LOGFILE="/tmp/carla_simworld.log"

DEFAULT_MAP="Town10HD"
DEFAULT_RESX=1280
DEFAULT_RESY=720
CARLA_PORT=2000
AIRSIM_PORT=41451
QUALITY="Low"
TEXTURE_POOL=2048

# ========================== Help ==========================
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<'EOF'
carlaAir.sh — Unified CARLA + AirSim Launcher

Usage: ./carlaAir.sh [MAP] [ResX] [ResY] [OPTIONS]

Arguments:
  MAP          Map name (default: Town10HD)
               Available: Town01-05, Town10HD (and _Opt variants)
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
EOF
    exit 0
fi

# ========================== Kill mode ==========================
if [[ "$1" == "--kill" ]]; then
    echo "Stopping CARLA+AirSim..."
    pkill -f "UE4Editor.*CarlaUE4" 2>/dev/null && echo "Stopped." || echo "Not running."
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
        Town*) MAP="$arg" ;;
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
# Check if already running (use ss to check port — more reliable than pgrep)
if ss -tlnp 2>/dev/null | grep -q ":${CARLA_PORT} "; then
    echo "WARNING: Port $CARLA_PORT is already in use (CARLA likely running)!"
    echo "  Use './carlaAir.sh --kill' to stop it first."
    exit 1
fi

# Check display
if [[ -z "$DISPLAY" ]]; then
    export DISPLAY=:1
fi

# Vulkan ICD
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

# Verify paths
if [[ ! -f "$UE4_ROOT/Engine/Binaries/Linux/UE4Editor" ]]; then
    echo "ERROR: UE4Editor not found at $UE4_ROOT"
    exit 1
fi

if [[ ! -f "$PROJECT" ]]; then
    echo "ERROR: Project not found at $PROJECT"
    exit 1
fi

# ========================== Launch ==========================
MAP_PATH="/Game/Carla/Maps/${MAP}"

CMD=(
    "$UE4_ROOT/Engine/Binaries/Linux/UE4Editor"
    "$PROJECT"
    "$MAP_PATH"
    -game
    -windowed
    -ResX="$RESX"
    -ResY="$RESY"
    -carla-rpc-port="$CARLA_PORT"
    -quality-level="$QUALITY"
    -unattended
    -nosound
    -TexturePoolSize="$TEXTURE_POOL"
)

echo "============================================"
echo "  CARLA + AirSim Unified Simulator"
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

    # Wait for both ports
    MAX_WAIT=600
    ELAPSED=0
    CARLA_READY=false
    AIRSIM_READY=false

    while [[ $ELAPSED -lt $MAX_WAIT ]]; do
        # Check if process died
        if ! kill -0 $PID 2>/dev/null; then
            echo "ERROR: UE4Editor crashed during startup!"
            echo "Check log: tail -50 $LOGFILE"
            exit 1
        fi

        # Check ports
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
            echo "Stop:  ./carlaAir.sh --kill"
            echo "Log:   ./carlaAir.sh --log"
            exit 0
        fi

        sleep 5
        ELAPSED=$((ELAPSED + 5))

        # Progress indicator every 30s
        if [[ $((ELAPSED % 30)) -eq 0 ]]; then
            LAST_LOG=$(tail -1 "$LOGFILE" 2>/dev/null | head -c 120)
            echo "  [$(date +%H:%M:%S)] Waiting... (${ELAPSED}s) $LAST_LOG"
        fi
    done

    echo "WARNING: Timed out after ${MAX_WAIT}s. Servers may still be starting."
    echo "Check: ss -tlnp | grep -E '$CARLA_PORT|$AIRSIM_PORT'"
    echo "Log:   tail -f $LOGFILE"
fi
