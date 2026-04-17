#!/usr/bin/env bash
# ============================================
#  CarlaAir v0.1.7 - Air-Ground Co-Simulation Platform
#  CARLA + AirSim Unified Launcher
# ============================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BINARY="${SCRIPT_DIR}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
LOGFILE="${SCRIPT_DIR}/CarlaAir.log"

# Defaults
MAP="Town10HD"
RES_X=1280
RES_Y=720
CARLA_PORT=2000
AIRSIM_PORT=41451
QUALITY="Epic"
FG=0
USE_OPENGL=0

# Auto-detect Vulkan ICD files (support NVIDIA, AMD, Intel)
detect_vulkan_icd() {
    local ICD_DIRS="/usr/share/vulkan/icd.d /etc/vulkan/icd.d /usr/local/share/vulkan/icd.d"
    local ICD_FILES=""
    for dir in ${ICD_DIRS}; do
        if [ -d "${dir}" ]; then
            for f in "${dir}"/*.json; do
                [ -f "$f" ] && ICD_FILES="${ICD_FILES:+${ICD_FILES}:}$f"
            done
        fi
    done
    if [ -n "${ICD_FILES}" ]; then
        export VK_ICD_FILENAMES="${ICD_FILES}"
    fi
    # If nothing found, don't set VK_ICD_FILENAMES — let driver default work
}

usage() {
    cat <<EOF
Usage: $0 [MAP] [OPTIONS]

CarlaAir v0.1.7 - CARLA + AirSim Air-Ground Co-Simulation Platform

Arguments:
  MAP                Map name (default: Town10HD)
                     Available: Town01-05, Town10HD (and _Opt variants)

Options:
  --res WxH          Window resolution (default: 1280x720)
  --port PORT        CARLA RPC port (default: 2000)
  --quality LEVEL    Quality level: Low, Medium, High, Epic (default: Epic)
  --opengl           Use OpenGL4 instead of Vulkan
  --fg               Run in foreground (see all logs)
  --kill             Stop running CarlaAir instance
  --log              Show live log
  --help             Show this help

Built-in Controls (in UE4 window):
  WASD               Move drone (forward/back/strafe)
  Space / Shift      Fly up / down
  Mouse              Yaw (turn left/right)
  Scroll Wheel       Adjust drone speed
  N                  Cycle weather presets
  P                  Toggle physics/invincible mode
  H                  Show/hide help overlay
  Tab                Toggle mouse capture

Examples:
  $0                          # Start with Town10HD, Epic quality
  $0 Town03                   # Start with Town03
  $0 Town10HD --res 1920x1080 # Full HD
  $0 --opengl                 # Use OpenGL renderer
  $0 --kill                   # Stop running instance

After startup, connect with:
  CARLA:  python -c "import carla; c=carla.Client('localhost',${CARLA_PORT}); print(c.get_world())"
  AirSim: python -c "import airsim; c=airsim.MultirotorClient(port=${AIRSIM_PORT}); c.confirmConnection()"
EOF
}

# Parse arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --kill)
            pkill -f "auto_traffic.py" 2>/dev/null && echo "Traffic stopped." || true
            sleep 1
            pkill -f "CarlaUE4-Linux-Shipping" 2>/dev/null && echo "CarlaAir stopped." || echo "No running instance found."
            exit 0 ;;
        --log)
            [ -f "${LOGFILE}" ] && tail -f "${LOGFILE}" || echo "No log file found."
            exit 0 ;;
        --fg) FG=1 ;;
        --opengl) USE_OPENGL=1 ;;
        --res)
            shift; RES_X="${1%%x*}"; RES_Y="${1##*x}" ;;
        --port)
            shift; CARLA_PORT="$1" ;;
        --quality)
            shift; QUALITY="$1" ;;
        --help|-h) usage; exit 0 ;;
        Town*|town*) MAP="$1" ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
    shift
done

# Check binary
if [ ! -x "${BINARY}" ]; then
    chmod +x "${BINARY}" 2>/dev/null || true
    if [ ! -x "${BINARY}" ]; then
        echo "Error: Binary not found: ${BINARY}"
        exit 1
    fi
fi

# Check AirSim settings
AIRSIM_SETTINGS="${HOME}/Documents/AirSim/settings.json"
if [ ! -f "${AIRSIM_SETTINGS}" ]; then
    echo "Setting up AirSim configuration..."
    mkdir -p "${HOME}/Documents/AirSim"
    cp "${SCRIPT_DIR}/AirSimConfig/settings.json" "${AIRSIM_SETTINGS}"
    echo "  -> Created ${AIRSIM_SETTINGS}"
fi

# Auto-kill existing instance (no need for --kill)
if pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null 2>&1; then
    echo "Stopping previous CarlaAir instance..."
    pkill -9 -f "CarlaUE4-Linux-Shipping" 2>/dev/null
    sleep 3
    # Double-check
    if pgrep -f "CarlaUE4-Linux-Shipping" > /dev/null 2>&1; then
        echo "Warning: Could not stop previous instance. Trying harder..."
        pkill -9 -f "CarlaUE4" 2>/dev/null
        sleep 2
    fi
    echo "Previous instance stopped."
fi

# Set Vulkan environment (auto-detect ICD files)
if [ ${USE_OPENGL} -eq 0 ]; then
    detect_vulkan_icd
fi

# Build launch command
CMD=("${BINARY}" "CarlaUE4" "${MAP}")
CMD+=("-windowed" "-ResX=${RES_X}" "-ResY=${RES_Y}")
CMD+=("-carla-rpc-port=${CARLA_PORT}")
CMD+=("-quality-level=${QUALITY}")
CMD+=("-TexturePoolSize=2048")
CMD+=("-unattended" "-nosound" "-UseVSync")
if [ "${USE_OPENGL}" -eq 1 ]; then
    CMD+=("-opengl4")
fi
# CMD+=("-RenderOffScreen")  # Uncomment for headless mode

echo "============================================"
echo "  CarlaAir - Air-Ground Co-Simulation"
echo "  Version:   v0.1.7"
echo "============================================"
echo "  Map:        ${MAP}"
echo "  Resolution: ${RES_X}x${RES_Y}"
echo "  CARLA Port: ${CARLA_PORT}"
echo "  AirSim Port: ${AIRSIM_PORT}"
echo "  Quality:    ${QUALITY}"
echo "  Renderer:   $([ ${USE_OPENGL} -eq 1 ] && echo 'OpenGL4' || echo 'Vulkan')"
echo "============================================"

if [ "${FG}" -eq 1 ]; then
    exec "${CMD[@]}"
else
    nohup "${CMD[@]}" > "${LOGFILE}" 2>&1 &
    PID=$!
    echo "  PID: ${PID}"
    echo "  Log: ${LOGFILE}"
    echo ""
    echo "Waiting for ports..."

    # Wait for CARLA port
    for _ in $(seq 1 120); do
        if ss -tlnp 2>/dev/null | grep -q ":${CARLA_PORT} "; then
            echo "  CARLA (port ${CARLA_PORT}): Ready"
            break
        fi
        if ! kill -0 ${PID} 2>/dev/null; then
            echo "  Error: Process died. Check ${LOGFILE}"
            exit 1
        fi
        sleep 2
    done

    # Wait for AirSim port
    for _ in $(seq 1 60); do
        if ss -tlnp 2>/dev/null | grep -q ":${AIRSIM_PORT} "; then
            echo "  AirSim (port ${AIRSIM_PORT}): Ready"
            break
        fi
        sleep 2
    done

    # v0.1.7: Auto-spawn traffic (vehicles + pedestrians)
    TRAFFIC_SCRIPT="${SCRIPT_DIR}/auto_traffic.py"
    TRAFFIC_PYTHON=""
    # Find a working python3 with carla module
    # 1. Project venv
    if [ -x "${SCRIPT_DIR}/.venv/bin/python3" ]; then
        TRAFFIC_PYTHON="${SCRIPT_DIR}/.venv/bin/python3"
    # 2. Current python3 (from conda activate or system)
    elif command -v python3 &>/dev/null && python3 -c "import carla" 2>/dev/null; then
        TRAFFIC_PYTHON="python3"
    fi

    if [ -f "${TRAFFIC_SCRIPT}" ] && [ -n "${TRAFFIC_PYTHON}" ]; then
        echo ""
        echo "Spawning city traffic (30 vehicles + 50 walkers)..."
        nohup "${TRAFFIC_PYTHON}" "${TRAFFIC_SCRIPT}" \
            --vehicles 30 --walkers 50 --port "${CARLA_PORT}" \
            >> "${SCRIPT_DIR}/traffic.log" 2>&1 &
        sleep 8
        echo "  Traffic running in background."
    elif [ -f "${TRAFFIC_SCRIPT}" ] && [ -z "${TRAFFIC_PYTHON}" ]; then
        echo ""
        echo "  Warning: python3 with 'carla' module not found."
        echo "  Traffic not spawned. To fix:"
        echo "    conda activate carlaAir   # or run: bash env_setup/setup_env.sh"
        echo "  Then restart CarlaAir."
    fi

    echo ""
    echo "CarlaAir is ready! FPS drone control active in UE4 window."
    echo ""
    echo "  Built-in: WASD=Move  Mouse=Yaw  Scroll=Speed  N=Weather  P=Physics  H=Help"
    echo "  City traffic: 30 vehicles + 50 pedestrians (auto-spawned)"
    echo ""
    echo "  Stop: $0 --kill"
fi
