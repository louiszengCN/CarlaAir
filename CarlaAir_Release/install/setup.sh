#!/bin/bash
# setup.sh — CarlaAir 自动安装脚本
# 安装 Python 依赖、配置 AirSim、创建启动脚本

set -e
echo "============================================"
echo "  CarlaAir Setup Script"
echo "============================================"

# === Step 1: Python dependencies ===
echo ""
echo "[1/3] Checking Python dependencies..."

PYTHON=""
if command -v python3 &>/dev/null; then
    PYTHON=python3
elif command -v python &>/dev/null; then
    PYTHON=python
fi

if [ -z "$PYTHON" ]; then
    echo "ERROR: Python not found. Install Python 3.7+ first."
    exit 1
fi

echo "Python: $($PYTHON --version)"

# Check for pip
if ! $PYTHON -m pip --version &>/dev/null; then
    echo "ERROR: pip not found. Install pip first."
    exit 1
fi

# Install required packages
echo "Installing required Python packages..."
$PYTHON -m pip install --quiet carla==0.9.16 airsim numpy opencv-python 2>/dev/null || {
    echo "WARNING: Some packages failed to install. You may need to install them manually:"
    echo "  pip install carla==0.9.16 airsim numpy opencv-python"
}

# === Step 2: AirSim settings ===
echo ""
echo "[2/3] Configuring AirSim settings..."

SETTINGS_DIR=~/Documents/AirSim
SETTINGS_FILE=$SETTINGS_DIR/settings.json

if [ -f "$SETTINGS_FILE" ]; then
    echo "AirSim settings already exist: $SETTINGS_FILE"
    echo "Skipping (not overwriting existing settings)."
else
    mkdir -p "$SETTINGS_DIR"
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    if [ -f "$SCRIPT_DIR/settings.json" ]; then
        cp "$SCRIPT_DIR/settings.json" "$SETTINGS_FILE"
    else
        cat > "$SETTINGS_FILE" << 'EOF'
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
                    "X": 0.5, "Y": 0.0, "Z": 0.1
                }
            }
        }
    }
}
EOF
    fi
    echo "Created: $SETTINGS_FILE"
fi

# === Step 3: Make launch script executable ===
echo ""
echo "[3/3] Setting up launch scripts..."

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
chmod +x "$SCRIPT_DIR/carla_air.sh" 2>/dev/null || true
chmod +x "$SCRIPT_DIR/CarlaUE4.sh" 2>/dev/null || true

echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "To start CarlaAir:"
echo "  cd $SCRIPT_DIR"
echo "  ./carla_air.sh"
echo ""
echo "To verify installation:"
echo "  $PYTHON -c \"import carla; print('CARLA API: OK')\""
echo "  $PYTHON -c \"import airsim; print('AirSim API: OK')\""
