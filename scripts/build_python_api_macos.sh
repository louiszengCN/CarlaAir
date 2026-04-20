#!/usr/bin/env bash
# build_python_api_macos.sh — Build CARLA Python API for macOS ARM
#
# Builds libcarla_client.a + Recast/Detour, populates PythonAPI/carla/dependencies/,
# then builds and installs the carla wheel into the project venv.
#
# Prerequisites (already satisfied by build_carla_deps_macos.sh):
#   brew install boost boost-python3 libpng libjpeg libtiff sqlite xerces-c proj
#
# Usage:
#   bash scripts/build_python_api_macos.sh

set -euo pipefail

CARLA_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$CARLA_ROOT/Build/macos"
VENV="$CARLA_ROOT/.venv"
PYTHON="$VENV/bin/python3"
JOBS=$(sysctl -n hw.logicalcpu)

HOMEBREW_PREFIX="${HOMEBREW_PREFIX:-/opt/homebrew}"

# Python version suffix used by boost-python (e.g. "314" for 3.14)
PY_VER_SUFFIX="$("$PYTHON" -c 'import sys; print(f"{sys.version_info.major}{sys.version_info.minor}")')"

echo "=== CarlaAir Python API macOS build ==="
echo "Python: $("$PYTHON" --version)"
echo "Boost-Python lib: libboost_python${PY_VER_SUFFIX}.a"
echo ""

# ---- Recast/Detour ----
RECAST_SRC="$BUILD_DIR/recastnavigation"
RECAST_BUILD="$BUILD_DIR/recast-build"
RECAST_INSTALL="$BUILD_DIR/recast-install"

if [[ ! -d "$RECAST_SRC/.git" ]]; then
    echo "=== [1/3] Cloning Recast/Detour (carla branch) ==="
    git clone --depth=1 --branch carla \
        https://github.com/carla-simulator/recastnavigation.git "$RECAST_SRC"
fi

if [[ ! -f "$RECAST_INSTALL/lib/libRecast.a" ]]; then
    echo "=== [1/3] Building Recast/Detour ==="
    mkdir -p "$RECAST_BUILD"
    cmake -S "$RECAST_SRC" -B "$RECAST_BUILD" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DCMAKE_INSTALL_PREFIX="$RECAST_INSTALL" \
        -DRECASTNAVIGATION_DEMO=OFF \
        -DRECASTNAVIGATION_TESTS=OFF \
        -DRECASTNAVIGATION_EXAMPLES=OFF \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build "$RECAST_BUILD" --parallel "$JOBS"
    cmake --install "$RECAST_BUILD"
    echo "Recast/Detour built: $RECAST_INSTALL/lib/"
else
    echo "=== [1/3] Recast/Detour already built, skipping ==="
fi

# ---- libcarla_client ----
CLIENT_BUILD="$BUILD_DIR/libcarla-client-build"
CLIENT_INSTALL="$BUILD_DIR/libcarla-client-install"
RPCLIB_INSTALL="$BUILD_DIR/rpclib-install"

if [[ ! -f "$CLIENT_INSTALL/lib/libcarla_client.a" ]]; then
    echo "=== [2/3] Building libcarla_client ==="

    if [[ ! -d "$RPCLIB_INSTALL" ]]; then
        echo "ERROR: rpclib not built. Run build_carla_deps_macos.sh first."
        exit 1
    fi

    # cmake globs BOOST_LIB_PATH/* which would include gdk-pixbuf-2.0 etc. from
    # /opt/homebrew/lib — isolate only the boost libs we need.
    BOOST_ISOLATED="$BUILD_DIR/boost-isolated"
    mkdir -p "$BOOST_ISOLATED"
    cp "$HOMEBREW_PREFIX/lib/libboost_filesystem.a" "$BOOST_ISOLATED/" 2>/dev/null || true
    cp "$HOMEBREW_PREFIX/lib/libboost_program_options.a" "$BOOST_ISOLATED/" 2>/dev/null || true

    # cmake globs LIBPNG_LIB_PATH similarly — isolate libpng
    PNG_ISOLATED="$BUILD_DIR/png-isolated"
    mkdir -p "$PNG_ISOLATED"
    cp "$HOMEBREW_PREFIX/lib/libpng.a" "$PNG_ISOLATED/" 2>/dev/null || true
    cp "$HOMEBREW_PREFIX/include/png.h" "$PNG_ISOLATED/" 2>/dev/null || true

    mkdir -p "$CLIENT_BUILD"
    cmake -S "$CARLA_ROOT/LibCarla/cmake" -B "$CLIENT_BUILD" \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Client \
        -DCMAKE_OSX_ARCHITECTURES=arm64 \
        -DLIBCARLA_BUILD_RELEASE=ON \
        -DLIBCARLA_BUILD_DEBUG=OFF \
        -DLIBCARLA_BUILD_TEST=OFF \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG -DBOOST_NO_EXCEPTIONS -DASIO_NO_EXCEPTIONS -DLIBCARLA_WITH_PYTHON_SUPPORT" \
        -DRPCLIB_INCLUDE_PATH="$RPCLIB_INSTALL/include" \
        -DRPCLIB_LIB_PATH="$RPCLIB_INSTALL/lib" \
        -DBOOST_INCLUDE_PATH="$HOMEBREW_PREFIX/include" \
        -DBOOST_LIB_PATH="$BOOST_ISOLATED" \
        -DRECAST_INCLUDE_PATH="$RECAST_INSTALL/include" \
        -DRECAST_LIB_PATH="$RECAST_INSTALL/lib" \
        -DLIBPNG_INCLUDE_PATH="$HOMEBREW_PREFIX/include" \
        -DLIBPNG_LIB_PATH="$PNG_ISOLATED" \
        -DLLVM_LIB_PATH="" \
        -DCMAKE_INSTALL_PREFIX="$CLIENT_INSTALL"

    cmake --build "$CLIENT_BUILD" --parallel "$JOBS" --target install
    echo "libcarla_client built: $CLIENT_INSTALL/lib/libcarla_client.a"
else
    echo "=== [2/3] libcarla_client already built, skipping ==="
fi

# ---- Populate PythonAPI/carla/dependencies ----
echo "=== [3/3] Populating PythonAPI dependencies ==="
DEPS="$CARLA_ROOT/PythonAPI/carla/dependencies"
mkdir -p "$DEPS/include/system" "$DEPS/lib"

# Headers from client install
if [[ -d "$CLIENT_INSTALL/include" ]]; then
    cp -r "$CLIENT_INSTALL/include/." "$DEPS/include/" 2>/dev/null || true
fi

# libcarla_client
cp "$CLIENT_INSTALL/lib/libcarla_client.a" "$DEPS/lib/"

# rpclib
cp "$RPCLIB_INSTALL/lib/librpc.a" "$DEPS/lib/"
cp -r "$RPCLIB_INSTALL/include/rpc" "$DEPS/include/system/"

# Recast/Detour
for lib in Recast Detour DetourCrowd; do
    [[ -f "$RECAST_INSTALL/lib/lib${lib}.a" ]] && cp "$RECAST_INSTALL/lib/lib${lib}.a" "$DEPS/lib/"
done
[[ -d "$RECAST_INSTALL/include" ]] && cp -r "$RECAST_INSTALL/include" "$DEPS/include/system/recast"

# Homebrew libraries
for lib in boost_filesystem boost_program_options "boost_python${PY_VER_SUFFIX}" png; do
    src="$HOMEBREW_PREFIX/lib/lib${lib}.a"
    [[ -f "$src" ]] && cp "$src" "$DEPS/lib/" || echo "  WARNING: $src not found"
done

# OSM2ODR header (from sumo-carla build)
OSM2ODR_H="$BUILD_DIR/sumo-carla/src/OSM2ODR.h"
[[ -f "$OSM2ODR_H" ]] && cp "$OSM2ODR_H" "$DEPS/include/" || echo "  WARNING: OSM2ODR.h not found at $OSM2ODR_H"

# Carla server deps (from CarlaDependencies)
CARLA_DEPS="$CARLA_ROOT/Unreal/CarlaUE5/Plugins/Carla/CarlaDependencies"
for lib in osm2odr xerces-c proj sqlite3; do
    src="$CARLA_DEPS/lib/lib${lib}.a"
    [[ -f "$src" ]] && cp "$src" "$DEPS/lib/" || echo "  WARNING: $src not found"
done

# Boost headers (needed by libcarla headers)
if [[ -d "$HOMEBREW_PREFIX/include/boost" ]]; then
    mkdir -p "$DEPS/include/system"
    ln -sfn "$HOMEBREW_PREFIX/include/boost" "$DEPS/include/system/boost" 2>/dev/null || true
fi

echo "Dependencies:"
ls -lh "$DEPS/lib/"

# ---- Build the wheel ----
echo ""
echo "=== Building carla Python wheel ==="
cd "$CARLA_ROOT/PythonAPI/carla"

# Install build tools in venv
"$PYTHON" -m pip install --quiet setuptools wheel

"$PYTHON" setup.py bdist_wheel 2>&1 | tail -10

WHEEL=$(ls dist/carla-*.whl 2>/dev/null | head -1)
if [[ -z "$WHEEL" ]]; then
    echo "ERROR: No wheel built. Check output above."
    exit 1
fi
echo "Wheel built: $WHEEL"

# Install wheel into venv
"$PYTHON" -m pip install --force-reinstall "$WHEEL"

# Verify
"$PYTHON" -c "import carla; print('carla', carla.__version__, '— import OK')"
echo ""
echo "=== Python API build complete ==="
