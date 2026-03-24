#!/bin/bash
# ============================================
#  CarlaAir Environment Test
#  Checks: Python modules + server connectivity
# ============================================

PASS=0
FAIL=0
WARN=0

ok()   { echo "  [PASS] $1"; ((PASS++)); }
fail() { echo "  [FAIL] $1"; ((FAIL++)); }
warn() { echo "  [WARN] $1"; ((WARN++)); }

echo "============================================"
echo "  CarlaAir Environment Test"
echo "============================================"
echo ""

# ── Python ──
echo "── Python ──"
if command -v python3 &>/dev/null; then
    VER=$(python3 --version 2>&1)
    ok "python3 found: $VER"
else
    fail "python3 not found"
fi

# ── Module imports ──
echo ""
echo "── Python Modules ──"
for mod in carla airsim pygame numpy; do
    if python3 -c "import $mod" 2>/dev/null; then
        ok "$mod"
    else
        fail "$mod — not installed (run: bash env_setup/setup_env.sh)"
    fi
done

# Optional modules
for mod in cv2 PIL; do
    if python3 -c "import $mod" 2>/dev/null; then
        ok "$mod (optional)"
    else
        warn "$mod not installed (optional, needed for demo_director recording)"
    fi
done

# ── CARLA version check ──
echo ""
echo "── CARLA Compatibility ──"
python3 -c "
import carla
try:
    c = carla.Client('localhost', 2000)
    c.set_timeout(3.0)
    sv = c.get_server_version()
    cv = c.get_client_version()
    if sv == cv:
        print('  [PASS] Client and server versions match: ' + sv)
    else:
        print('  [WARN] Version mismatch: client=' + cv + ' server=' + sv)
        print('         This is expected with CarlaAir custom builds.')
except Exception as e:
    print('  [WARN] Cannot check version (server not running?): ' + str(e))
" 2>/dev/null

# ── Server connectivity ──
echo ""
echo "── Server Connectivity ──"

# CARLA port
if ss -tlnp 2>/dev/null | grep -q ':2000 '; then
    ok "CARLA port 2000 is listening"
    # Try get_world
    python3 -c "
import carla
try:
    c = carla.Client('localhost', 2000)
    c.set_timeout(5.0)
    w = c.get_world()
    m = w.get_map().name.split('/')[-1]
    print('  [PASS] CARLA world accessible: ' + m)
except Exception as e:
    print('  [FAIL] CARLA world not accessible: ' + str(e))
" 2>/dev/null
else
    warn "CARLA port 2000 not listening (start CarlaAir first)"
fi

# AirSim port
if ss -tlnp 2>/dev/null | grep -q ':41451 '; then
    ok "AirSim port 41451 is listening"
    python3 -c "
import airsim
try:
    c = airsim.MultirotorClient(ip='127.0.0.1', port=41451, timeout_value=5)
    if c.ping():
        print('  [PASS] AirSim RPC responsive')
    else:
        print('  [FAIL] AirSim ping returned false')
except Exception as e:
    print('  [FAIL] AirSim RPC failed: ' + str(e))
" 2>/dev/null
else
    warn "AirSim port 41451 not listening (start CarlaAir first)"
fi

# ── Summary ──
echo ""
echo "============================================"
echo "  Results: ${PASS} PASS, ${FAIL} FAIL, ${WARN} WARN"
if [ ${FAIL} -eq 0 ]; then
    echo "  Status: ALL CHECKS PASSED"
else
    echo "  Status: SOME CHECKS FAILED"
    echo "  Fix: conda activate carlaAir && bash env_setup/setup_env.sh"
fi
echo "============================================"

exit ${FAIL}
