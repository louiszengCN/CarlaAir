#!/bin/bash
# pack_simworld.sh — Package SimWorld for distribution
#
# Creates a tar.gz archive excluding debug symbols and unnecessary files.
# Output: SimWorld_<date>.tar.gz (~15GB → compressed ~12-14GB)
#
# Usage:
#   bash pack_simworld.sh                    # default output to current dir
#   bash pack_simworld.sh /path/to/output    # custom output directory

set -e

SRC="/mnt/data1/tianle/carla_source/Dist/CARLA_Shipping_1ae5356-dirty/LinuxNoEditor"
OUTDIR="${1:-.}"
DATE=$(date +%Y%m%d)
ARCHIVE="SimWorld_${DATE}.tar.gz"

echo "============================================="
echo "  SimWorld Packaging"
echo "============================================="
echo "  Source: ${SRC}"
echo "  Output: ${OUTDIR}/${ARCHIVE}"
echo ""

# Calculate size without excluded files
echo "[1] Calculating package size..."
FULL_SIZE=$(du -sh "${SRC}" | cut -f1)
DEBUG_SIZE=$(du -sh "${SRC}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping.debug" 2>/dev/null | cut -f1 || echo "0")
SYM_SIZE=$(du -sh "${SRC}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping.sym" 2>/dev/null | cut -f1 || echo "0")
echo "  Full directory: ${FULL_SIZE}"
echo "  Excluding debug symbols: ${DEBUG_SIZE} + ${SYM_SIZE}"
echo "  Excluding: Manifest files, __pycache__, .pyc, root-level test scripts"
echo ""

# Excluded patterns:
# - Debug symbols (1.7GB): *.debug, *.sym
# - Root-level test/demo scripts (users should use examples/ instead)
# - Python cache files
# - Manifest files (build metadata, not needed at runtime)
# - Dockerfile (not needed for direct usage)

echo "[2] Creating archive (this will take a while)..."
echo "    Compressing with gzip..."

mkdir -p "${OUTDIR}"

tar czf "${OUTDIR}/${ARCHIVE}" \
    -C "$(dirname "${SRC}")" \
    --transform="s/^LinuxNoEditor/SimWorld/" \
    --exclude="*.debug" \
    --exclude="*.sym" \
    --exclude="__pycache__" \
    --exclude="*.pyc" \
    --exclude="LinuxNoEditor/Manifest_*" \
    --exclude="LinuxNoEditor/Dockerfile" \
    --exclude="LinuxNoEditor/comprehensive_api_test.py" \
    --exclude="LinuxNoEditor/demo_drive_and_fly.py" \
    --exclude="LinuxNoEditor/demo_flight_city.py" \
    --exclude="LinuxNoEditor/test_package.py" \
    "LinuxNoEditor"

FINAL_SIZE=$(du -sh "${OUTDIR}/${ARCHIVE}" | cut -f1)
echo ""
echo "[3] Done!"
echo "    Archive: ${OUTDIR}/${ARCHIVE}"
echo "    Size: ${FINAL_SIZE}"
echo ""
echo "To distribute:"
echo "  1. Copy ${ARCHIVE} to target machine"
echo "  2. Extract:  tar xzf ${ARCHIVE}"
echo "  3. Setup:    mkdir -p ~/Documents/AirSim && cp SimWorld/AirSimConfig/settings.json.example ~/Documents/AirSim/settings.json"
echo "  4. Launch:   cd SimWorld && ./SimWorld.sh"
echo "  5. Test:     python3 examples/drive_car.py"
