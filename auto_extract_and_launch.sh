#!/bin/bash
# Auto-extract CARLA content and launch UE4 with Town01 + AirSim
# This script waits for the Content.tar.gz download to complete, then:
# 1. Verifies the archive integrity
# 2. Extracts to Content/Carla
# 3. Launches UE4 with Town01 map and AirSim game mode

set -e

CARLA_SOURCE="/mnt/data1/tianle/carla_source"
CONTENT_FILE="${CARLA_SOURCE}/Content.tar.gz"
CONTENT_DIR="${CARLA_SOURCE}/Unreal/CarlaUE4/Content/Carla"
UE4_ROOT="/mnt/data1/tianle/carla_ue4"
LOG_FILE="/tmp/carla_town01.log"
EXPECTED_SIZE=21567002106

echo "=== CARLA Content Auto-Extract & Launch Script ==="
echo "Waiting for Content.tar.gz download to complete..."
echo "Expected size: ${EXPECTED_SIZE} bytes ($(( EXPECTED_SIZE / 1024 / 1024 / 1024 )) GB)"

# Wait for download to complete
while true; do
    if [ ! -f "$CONTENT_FILE" ]; then
        echo "Content.tar.gz not found, waiting..."
        sleep 30
        continue
    fi

    CURRENT_SIZE=$(stat -c %s "$CONTENT_FILE" 2>/dev/null || echo 0)
    PERCENT=$(( CURRENT_SIZE * 100 / EXPECTED_SIZE ))
    REMAINING_MB=$(( (EXPECTED_SIZE - CURRENT_SIZE) / 1024 / 1024 ))
    echo "Progress: ${PERCENT}% (${REMAINING_MB} MB remaining)"

    if [ "$CURRENT_SIZE" -ge "$EXPECTED_SIZE" ]; then
        # Check if wget is still writing
        if ! pgrep -f "wget.*Content.tar.gz" > /dev/null 2>&1; then
            echo "Download complete!"
            break
        fi
        echo "File is full size but wget still running, waiting..."
    fi

    sleep 60
done

# Verify archive integrity
echo ""
echo "=== Step 1: Verifying archive integrity ==="
if tar -tzf "$CONTENT_FILE" > /dev/null 2>&1; then
    echo "Archive is valid!"
else
    echo "WARNING: Archive may be corrupted, but attempting extraction anyway..."
fi

# Extract content
echo ""
echo "=== Step 2: Extracting content ==="
echo "Target: ${CONTENT_DIR}"
mkdir -p "$CONTENT_DIR"

# Extract (the archive contains files without a top-level directory)
cd "$CONTENT_DIR"
tar -xzf "$CONTENT_FILE" 2>&1
echo "Extraction complete!"

# Verify key files
echo ""
echo "=== Step 3: Verifying extraction ==="
if [ -f "${CONTENT_DIR}/Maps/Town01.umap" ]; then
    echo "Town01.umap found!"
else
    echo "WARNING: Town01.umap not found!"
fi

TOTAL_SIZE=$(du -sh "$CONTENT_DIR" | cut -f1)
echo "Total content size: ${TOTAL_SIZE}"

# Clean up archive
echo "Removing archive to save space..."
rm -f "$CONTENT_FILE"

# Kill standalone CARLA if running
echo ""
echo "=== Step 4: Stopping standalone CARLA ==="
pkill -f "CarlaUE4-Linux-Shipping" 2>/dev/null || true
sleep 2

# Launch UE4 with Town01 + AirSim
echo ""
echo "=== Step 5: Launching UE4 with Town01 + AirSim ==="
export DISPLAY=:1
export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

nohup ${UE4_ROOT}/Engine/Binaries/Linux/UE4Editor \
  "${CARLA_SOURCE}/Unreal/CarlaUE4/CarlaUE4.uproject" \
  "/Game/Carla/Maps/Town01?game=/Script/AirSim.AirSimGameMode" \
  -game -windowed -ResX=1280 -ResY=720 \
  -carla-rpc-port=2000 \
  -quality-level=Low \
  -unattended \
  -nosound \
  > "$LOG_FILE" 2>&1 &

UE4_PID=$!
echo "UE4 started with PID: ${UE4_PID}"
echo "Log file: ${LOG_FILE}"
echo ""
echo "=== Setup complete! ==="
echo "Wait ~2-3 minutes for the map to load, then test with:"
echo "  python3 test_drone_in_carla.py"
