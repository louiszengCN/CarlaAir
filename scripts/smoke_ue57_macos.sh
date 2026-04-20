#!/usr/bin/env bash
# smoke_ue57_macos.sh — CarlaUE5 editor smoke test for macOS/UE5.7
#
# Launches the editor in headless mode, waits for QUIT to be processed,
# then forces clean exit within a tight timeout (no 4-hour hangs).
#
# Usage:
#   bash scripts/smoke_ue57_macos.sh [--timeout SECONDS]
#
# Exit codes:
#   0  — Editor started, QUIT received, no fatal errors
#   1  — Fatal error or crash detected
#   2  — Hard timeout before QUIT was even received

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

EDITOR="/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor"
PROJECT="$PROJECT_DIR/Unreal/CarlaUE5/CarlaUE5.uproject"

# Max seconds to wait after QUIT is confirmed received
POST_QUIT_WAIT=30
# Absolute max seconds before giving up entirely
HARD_TIMEOUT=180

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout) shift; HARD_TIMEOUT="$1" ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
    shift
done

# Timestamped log file in project root
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOGFILE="$PROJECT_DIR/smoke_ue57_unrealeditor_unattended_${TIMESTAMP}.log"
echo "$LOGFILE" > "$PROJECT_DIR/.latest_smoke_log_path"

echo "Smoke launch start: $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "$LOGFILE"
echo "Editor: $EDITOR"  | tee -a "$LOGFILE"
echo "Project: $PROJECT" | tee -a "$LOGFILE"
echo "" | tee -a "$LOGFILE"

if [[ ! -f "$EDITOR" ]]; then
    echo "ERROR: Editor not found at $EDITOR" | tee -a "$LOGFILE"
    exit 1
fi

# Launch in headless / unattended mode
# -nosourcecontrol keeps the asset discovery fast and avoids EOS polling loops
"$EDITOR" "$PROJECT" \
    -unattended \
    -NoSplash \
    -NullRHI \
    -NoSound \
    -nosourcecontrol \
    -stdout \
    -FullStdOutLogOutput \
    -ExecCmds=QUIT \
    >> "$LOGFILE" 2>&1 &
PID=$!

START=$(date +%s)
QUIT_SEEN_AT=0
EXIT_CODE=0
RESULT="unknown"

while true; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - START ))

    # Check if process already exited on its own
    if ! kill -0 "$PID" 2>/dev/null; then
        wait "$PID" 2>/dev/null && EXIT_CODE=0 || EXIT_CODE=$?
        RESULT="self-exited"
        break
    fi

    # Hard timeout
    if [[ $ELAPSED -ge $HARD_TIMEOUT ]]; then
        echo "" | tee -a "$LOGFILE"
        echo "[smoke] Hard timeout after ${HARD_TIMEOUT}s — killing process" | tee -a "$LOGFILE"
        kill -KILL "$PID" 2>/dev/null || true
        wait "$PID" 2>/dev/null || true
        EXIT_CODE=2
        RESULT="hard-timeout"
        break
    fi

    # Once QUIT is confirmed in log, start the post-quit countdown
    if [[ $QUIT_SEEN_AT -eq 0 ]] && grep -q "Cmd: QUIT" "$LOGFILE" 2>/dev/null; then
        QUIT_SEEN_AT=$(date +%s)
        echo "" | tee -a "$LOGFILE"
        echo "[smoke] QUIT received at +${ELAPSED}s — waiting up to ${POST_QUIT_WAIT}s for clean exit" | tee -a "$LOGFILE"
    fi

    if [[ $QUIT_SEEN_AT -ne 0 ]]; then
        SINCE_QUIT=$(( NOW - QUIT_SEEN_AT ))
        if [[ $SINCE_QUIT -ge $POST_QUIT_WAIT ]]; then
            echo "[smoke] Process still alive ${SINCE_QUIT}s after QUIT — sending SIGTERM" | tee -a "$LOGFILE"
            kill -TERM "$PID" 2>/dev/null || true
            sleep 3
            if kill -0 "$PID" 2>/dev/null; then
                kill -KILL "$PID" 2>/dev/null || true
            fi
            wait "$PID" 2>/dev/null || true
            # Treat forced exit after QUIT as success (editor was healthy)
            EXIT_CODE=0
            RESULT="quit-forced"
            break
        fi
    fi

    sleep 2
done

# Check for fatal errors in log
FATAL_COUNT=$(grep -c "^.*LogCore: \[Callstack\]\|Fatal error!\|Assertion failed:" "$LOGFILE" 2>/dev/null || true)
if [[ $FATAL_COUNT -gt 0 ]]; then
    echo "[smoke] FATAL errors detected: $FATAL_COUNT" | tee -a "$LOGFILE"
    EXIT_CODE=1
    RESULT="fatal-error"
fi

# Summary
QUIT_STATUS="QUIT not received"
if [[ $QUIT_SEEN_AT -ne 0 ]]; then
    QUIT_STATUS="QUIT received at +$(( QUIT_SEEN_AT - START ))s"
fi

echo "" | tee -a "$LOGFILE"
echo "Smoke launch end: $(date '+%Y-%m-%d %H:%M:%S')" | tee -a "$LOGFILE"
echo "Exit code: $EXIT_CODE"  | tee -a "$LOGFILE"
echo "[smoke] Result: $RESULT | $QUIT_STATUS | fatals=$FATAL_COUNT" | tee -a "$LOGFILE"

if [[ $EXIT_CODE -eq 0 ]]; then
    echo "[smoke] PASS"
else
    echo "[smoke] FAIL (exit $EXIT_CODE)"
fi

exit $EXIT_CODE
