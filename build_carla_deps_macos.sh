#!/usr/bin/env bash
# Build CARLA server dependencies for macOS ARM (Apple Silicon)
# Produces: librpc.a, libcarla_server.a + headers in CarlaDependencies/
set -euo pipefail

CARLA_ROOT="$(cd "$(dirname "$0")" && pwd)"
UE4_ROOT="$CARLA_ROOT/Unreal/CarlaUE5"
PLUGIN_ROOT="$UE4_ROOT/Plugins/Carla"
CARLA_DEPS="$PLUGIN_ROOT/CarlaDependencies"
BUILD_DIR="$CARLA_ROOT/Build/macos"

# Allow callers to override Homebrew prefix (e.g. Intel Mac: /usr/local)
HOMEBREW_PREFIX="${HOMEBREW_PREFIX:-/opt/homebrew}"

BOOST_INCLUDE="$HOMEBREW_PREFIX/include"
SQLITE_PREFIX="$HOMEBREW_PREFIX/opt/sqlite"
XERCES_LIB="$HOMEBREW_PREFIX/lib/libxerces-c.a"
XERCES_INCLUDE="$HOMEBREW_PREFIX/include"
PROJ_LIB="$HOMEBREW_PREFIX/lib/libproj.a"
PROJ_INCLUDE="$HOMEBREW_PREFIX/include"

# Validate required Homebrew packages are installed
for req_path in "$BOOST_INCLUDE/boost/version.hpp" "$SQLITE_PREFIX/lib/libsqlite3.a" \
                "$XERCES_LIB" "$PROJ_LIB"; do
    if [ ! -e "$req_path" ]; then
        echo "ERROR: Missing dependency: $req_path"
        echo "Run: brew install boost sqlite xerces-c proj"
        exit 1
    fi
done

JOBS=$(sysctl -n hw.logicalcpu)

mkdir -p "$BUILD_DIR" "$CARLA_DEPS/lib" "$CARLA_DEPS/include"

echo "=== [1/3] Building rpclib ==="
RPCLIB_SRC="$BUILD_DIR/rpclib"
RPCLIB_INSTALL="$BUILD_DIR/rpclib-install"

if [ ! -d "$RPCLIB_SRC/.git" ]; then
  git clone --depth=1 --branch carla-callbacks \
    https://github.com/carla-simulator/rpclib.git "$RPCLIB_SRC"
fi

mkdir -p "$BUILD_DIR/rpclib-build"
cmake -S "$RPCLIB_SRC" -B "$BUILD_DIR/rpclib-build" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_INSTALL_PREFIX="$RPCLIB_INSTALL" \
  -DCMAKE_CXX_STANDARD=14 \
  -DRPCLIB_ENABLE_TESTS=OFF

cmake --build "$BUILD_DIR/rpclib-build" --parallel "$JOBS"
cmake --install "$BUILD_DIR/rpclib-build"

cp "$RPCLIB_INSTALL/lib/librpc.a" "$CARLA_DEPS/lib/"
cp -r "$RPCLIB_INSTALL/include/"* "$CARLA_DEPS/include/"
echo "rpclib built: $CARLA_DEPS/lib/librpc.a"

echo "=== [2/3] Building libcarla_server ==="
CARLA_BUILD_DIR="$BUILD_DIR/libcarla-server-build"
mkdir -p "$CARLA_BUILD_DIR"

cmake -S "$CARLA_ROOT/LibCarla/cmake" -B "$CARLA_BUILD_DIR" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Server \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DLIBCARLA_BUILD_RELEASE=ON \
  -DLIBCARLA_BUILD_DEBUG=OFF \
  -DLIBCARLA_BUILD_TEST=OFF \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG -DBOOST_NO_EXCEPTIONS -DASIO_NO_EXCEPTIONS -DLIBCARLA_NO_EXCEPTIONS -DBOOST_DISABLE_ABI_HEADERS -DBOOST_TYPE_INDEX_FORCE_NO_RTTI_COMPATIBILITY" \
  -DRPCLIB_INCLUDE_PATH="$RPCLIB_INSTALL/include" \
  -DRPCLIB_LIB_PATH="$RPCLIB_INSTALL/lib" \
  -DBOOST_INCLUDE_PATH="$BOOST_INCLUDE" \
  -DLLVM_LIB_PATH="" \
  -DCMAKE_INSTALL_PREFIX="$CARLA_DEPS"

cmake --build "$CARLA_BUILD_DIR" --parallel "$JOBS" --target install
echo "libcarla_server built."

echo "=== [3/3] Building osm2odr ==="
SUMO_COMMIT="1da3c07e"
SUMO_SRC="$BUILD_DIR/sumo-carla"
OSM2ODR_BUILD="$BUILD_DIR/osm2odr-build"

if [ ! -d "$SUMO_SRC/.git" ]; then
  # Use --filter=blob:none (treeless clone) to avoid downloading all blobs upfront,
  # while still being able to reach any commit. Plain --depth=N can miss older commits.
  git clone --filter=blob:none --no-checkout https://github.com/carla-simulator/sumo.git "$SUMO_SRC"
  git -C "$SUMO_SRC" checkout "$SUMO_COMMIT"
fi

# Find the osm2odr CMakeLists.txt
OSM2ODR_CMAKE=""
for candidate in \
  "$SUMO_SRC/src/netimport/osm/osmimporter" \
  "$SUMO_SRC/osm2odr" \
  "$SUMO_SRC"; do
  if [ -f "$candidate/CMakeLists.txt" ]; then
    OSM2ODR_CMAKE="$candidate"
    break
  fi
done

if [ -z "$OSM2ODR_CMAKE" ]; then
  echo "WARNING: Could not find osm2odr CMakeLists.txt, skipping"
else
  mkdir -p "$OSM2ODR_BUILD"
  cmake -S "$OSM2ODR_CMAKE" -B "$OSM2ODR_BUILD" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_CXX_STANDARD=17 \
    -DSQLITE3_INCLUDE_DIRS="$SQLITE_PREFIX/include" \
    -DSQLITE3_LIBRARIES="$SQLITE_PREFIX/lib/libsqlite3.a" \
    -DXercesC_INCLUDE_DIRS="$XERCES_INCLUDE" \
    -DXercesC_LIBRARIES="$XERCES_LIB" \
    -DPROJ_INCLUDE_DIRS="$PROJ_INCLUDE" \
    -DPROJ_LIBRARIES="$PROJ_LIB" \
    -DCMAKE_INSTALL_PREFIX="$CARLA_DEPS"

  cmake --build "$OSM2ODR_BUILD" --parallel "$JOBS" --target install || true
fi

# Copy sqlite3 static lib (Carla.Build.cs looks for it in CarlaDependencies/lib)
cp "$SQLITE_PREFIX/lib/libsqlite3.a" "$CARLA_DEPS/lib/" || true

echo ""
echo "=== Done ==="
ls -lh "$CARLA_DEPS/lib/"
