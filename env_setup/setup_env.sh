#!/bin/bash
# ============================================================
#  CarlaAir v0.1.7 — One-click Environment Setup
#  Usage: bash setup_env.sh
# ============================================================
set -e

ENV_NAME="carlaAir"
PYTHON_VER="3.10"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARBALL="$SCRIPT_DIR/carla_python_module.tar.gz"

# ---------- colors ----------
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[!!]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }

echo ""
echo "=========================================="
echo "  CarlaAir v0.1.7 Environment Setup"
echo "=========================================="
echo ""

# ---------- 1. Find conda ----------
CONDA_EXE=""
for p in "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda" "$(which conda 2>/dev/null)"; do
    if [ -x "$p" ]; then CONDA_EXE="$p"; break; fi
done
[ -z "$CONDA_EXE" ] && fail "conda not found. Install miniconda first: https://docs.conda.io/en/latest/miniconda.html"
ok "conda found: $CONDA_EXE"

# Source conda shell functions
CONDA_BASE="$("$CONDA_EXE" info --base)"
source "$CONDA_BASE/etc/profile.d/conda.sh"

# ---------- 2. Check tarball ----------
[ ! -f "$TARBALL" ] && fail "carla_python_module.tar.gz not found in $SCRIPT_DIR"
ok "carla module tarball found ($(du -h "$TARBALL" | cut -f1))"

# ---------- 3. Create or reuse conda env ----------
if conda env list | grep -qw "$ENV_NAME"; then
    warn "conda env '$ENV_NAME' already exists, will update it"
else
    echo "Creating conda env '$ENV_NAME' (python $PYTHON_VER)..."
    conda create -n "$ENV_NAME" python="$PYTHON_VER" -y -q
    ok "conda env '$ENV_NAME' created"
fi

conda activate "$ENV_NAME"
ok "activated env: $ENV_NAME ($(python3 --version))"

# ---------- 4. Install pip packages ----------
echo "Installing pip dependencies..."
pip install -q pygame airsim numpy Pillow
ok "pip packages installed"

# ---------- 5. Remove official carla if present ----------
# Remove from conda env
if pip show carla &>/dev/null; then
    warn "Removing pip-installed carla from conda env (incompatible with CarlaAir)..."
    pip uninstall carla -y -q
    ok "official carla package removed from conda env"
fi
# Also remove from --user site if present (it shadows conda packages)
LOCAL_SITE="$HOME/.local/lib/python${PYTHON_VER}/site-packages"
if [ -d "$LOCAL_SITE/carla" ] && pip show --user carla &>/dev/null 2>&1; then
    warn "Removing pip-installed carla from ~/.local ..."
    pip uninstall carla -y -q 2>/dev/null || true
fi

# ---------- 6. Install CarlaAir carla module ----------
SITE_PKG="$(python3 -c 'import site; print(site.getsitepackages()[0])')"

# Clean old carla/ and carla.libs/ in target
for d in "$SITE_PKG/carla" "$SITE_PKG/carla.libs"; do
    [ -d "$d" ] && rm -rf "$d"
done

echo "Installing CarlaAir carla module to $SITE_PKG ..."
tar xzf "$TARBALL" -C "$SITE_PKG"
ok "carla + carla.libs installed ($(ls "$SITE_PKG/carla.libs/" | wc -l) shared libs)"

# ---------- 7. Verify ----------
echo ""
echo "Verifying imports..."

VERIFY_OK=true

python3 -c "
import carla
print('  carla loaded from:', carla.__file__)
c = carla.Client('localhost', 9999)
print('  client version:', c.get_client_version())
" 2>/dev/null && ok "carla verified" || {
    # Import might work even if no server — test import alone
    python3 -c "import carla; print('  carla import OK')" 2>&1 && ok "carla import OK (no server running)" || { fail "carla import failed!"; VERIFY_OK=false; }
}

python3 -c "import airsim; print('  airsim OK')" && ok "airsim verified" || { warn "airsim import failed"; VERIFY_OK=false; }
python3 -c "import pygame" 2>/dev/null && ok "pygame verified" || { warn "pygame import failed"; VERIFY_OK=false; }
python3 -c "import numpy" && ok "numpy verified" || { warn "numpy import failed"; VERIFY_OK=false; }
python3 -c "import PIL" && ok "Pillow verified" || { warn "Pillow import failed"; VERIFY_OK=false; }

# ---------- Done ----------
echo ""
if $VERIFY_OK; then
    echo "=========================================="
    echo -e "  ${GREEN}Setup complete!${NC}"
    echo "=========================================="
else
    echo "=========================================="
    echo -e "  ${YELLOW}Setup complete with warnings${NC}"
    echo "=========================================="
fi
echo ""
echo "  Usage:"
echo "    conda activate $ENV_NAME"
echo "    cd $SCRIPT_DIR"
echo "    ./CarlaAir.sh                        # Start simulator"
echo "    python3 examples_record_demo/record_drone.py"
echo ""
