#!/usr/bin/env bash
# run_carlaair.sh — Launch CARLA server and run integration tests
#
# Usage:
#   bash scripts/run_carlaair.sh [--timeout SECONDS] [--port PORT] [--server-only] [--render-offscreen] [--map MAP]
#
# Exit codes:
#   0  — All integration tests passed
#   1  — Tests failed or server didn't start
#   2  — Hard timeout waiting for server

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

EDITOR="/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor"
PROJECT="$PROJECT_DIR/Unreal/CarlaUE5/CarlaUE5.uproject"
PYTHON="$PROJECT_DIR/.venv/bin/python3"

CARLA_PORT=2000
SERVER_WAIT=120   # seconds to wait for server to become ready
SERVER_ONLY=false
RENDER_MODE="nullrhi"
START_MAP=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout)   shift; SERVER_WAIT="$1" ;;
        --port)      shift; CARLA_PORT="$1" ;;
        --server-only) SERVER_ONLY=true ;;
        --render-offscreen) RENDER_MODE="offscreen" ;;
        --map)       shift; START_MAP="$1" ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOGFILE="$PROJECT_DIR/carlaair_server_${TIMESTAMP}.log"
echo "$LOGFILE" > "$PROJECT_DIR/.latest_build_log_path"

cleanup() {
    if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
        echo ""
        echo "[run] Shutting down CARLA server (pid=$SERVER_PID)..."
        kill -TERM "$SERVER_PID" 2>/dev/null || true
        sleep 3
        kill -KILL "$SERVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "=== CarlaAir — Launch + Integration Test ==="
echo "Project : $PROJECT"
echo "Port    : $CARLA_PORT"
echo "Render  : $RENDER_MODE"
if [[ -n "$START_MAP" ]]; then
    echo "Map     : $START_MAP"
fi
echo "Log     : $LOGFILE"
echo ""

if [[ ! -f "$EDITOR" ]]; then
    echo "ERROR: UnrealEditor not found at $EDITOR"
    exit 1
fi

# ── Launch server ──────────────────────────────────────────────────────────────
echo "[1/3] Launching CARLA server (headless)..."
LAUNCH_ARGS=(
    -game
    -nosound
    -NoSplash
    -unattended
    -nosourcecontrol
    -stdout
    -FullStdOutLogOutput
    -carla-rpc-port="$CARLA_PORT"
)

if [[ "$RENDER_MODE" == "offscreen" ]]; then
    LAUNCH_ARGS+=( -RenderOffScreen )
else
    LAUNCH_ARGS+=( -NullRHI )
fi

if [[ -n "$START_MAP" ]]; then
    "$EDITOR" "$PROJECT" "/Game/Carla/Maps/$START_MAP" "${LAUNCH_ARGS[@]}" >> "$LOGFILE" 2>&1 &
else
    "$EDITOR" "$PROJECT" "${LAUNCH_ARGS[@]}" >> "$LOGFILE" 2>&1 &
fi
SERVER_PID=$!
echo "      Server PID: $SERVER_PID"

# ── Wait for port ──────────────────────────────────────────────────────────────
echo "[2/3] Waiting for server on port $CARLA_PORT (up to ${SERVER_WAIT}s)..."
START=$(date +%s)
while true; do
    if nc -z localhost "$CARLA_PORT" 2>/dev/null; then
        ELAPSED=$(( $(date +%s) - START ))
        echo "      Server ready after ${ELAPSED}s"
        break
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "ERROR: Server process exited before becoming ready."
        echo "Last 20 log lines:"
        tail -20 "$LOGFILE"
        exit 1
    fi
    if [[ $(( $(date +%s) - START )) -ge $SERVER_WAIT ]]; then
        echo "ERROR: Timed out waiting for server after ${SERVER_WAIT}s"
        tail -20 "$LOGFILE"
        exit 2
    fi
    sleep 2
done

echo "      Waiting for CARLA API handshake..."
HANDSHAKE_DEADLINE=$(( $(date +%s) + SERVER_WAIT ))
while true; do
    if "$PYTHON" -c "import carla; client=carla.Client('localhost', int('$CARLA_PORT')); client.set_timeout(5.0); print(client.get_server_version()); world=client.get_world(); print(world.get_map().name)" >/dev/null 2>&1; then
        ELAPSED=$(( $(date +%s) - START ))
        echo "      CARLA API ready after ${ELAPSED}s"
        break
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "ERROR: Server process exited before CARLA API became ready."
        echo "Last 20 log lines:"
        tail -20 "$LOGFILE"
        exit 1
    fi
    if [[ $(date +%s) -ge $HANDSHAKE_DEADLINE ]]; then
        echo "ERROR: Timed out waiting for CARLA API after ${SERVER_WAIT}s"
        tail -20 "$LOGFILE"
        exit 2
    fi
    sleep 2
done

if $SERVER_ONLY; then
    echo ""
    echo "Server is running. Ctrl-C to stop."
    wait "$SERVER_PID" || true
    exit 0
fi

# Give the server an extra moment to fully initialise the world
sleep 3

# ── Run integration tests ──────────────────────────────────────────────────────
echo "[3/3] Running integration tests..."
echo ""
"$PYTHON" "$SCRIPT_DIR/integration_test.py" --host localhost --port "$CARLA_PORT" --timeout 30
TEST_EXIT=$?

echo ""
if [[ $TEST_EXIT -eq 0 ]]; then
    echo "=== CarlaAir PASS ==="
else
    echo "=== CarlaAir FAIL (tests exit $TEST_EXIT) ==="
    echo "Server log tail:"
    tail -30 "$LOGFILE"
fi

exit $TEST_EXIT
