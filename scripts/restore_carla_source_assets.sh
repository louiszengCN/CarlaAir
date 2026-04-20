#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTENT_VERSIONS="$REPO_ROOT/Util/ContentVersions.txt"
DEFAULT_VERSION="0.9.16"
ASSET_BASE_URL="https://carla-assets.s3.us-east-005.backblazeb2.com"
IMPORT_CACHE_DIR="$REPO_ROOT/Import/cache"
TARGET_CONTENT_DIR="$REPO_ROOT/Unreal/CarlaUE5/Content/Carla"
MIN_FREE_GB=30
DOWNLOAD_ONLY=false
SKIP_DOWNLOAD=false
FORCE_EXTRACT=false
VERSION="$DEFAULT_VERSION"
ARCHIVE_PATH=""

usage() {
  cat <<'EOF'
Usage: scripts/restore_carla_source_assets.sh [options]

Restores the raw CARLA source content archive into Unreal/CarlaUE5/Content/Carla.

Options:
  --version VERSION      Content version from Util/ContentVersions.txt (default: 0.9.16)
  --archive PATH         Existing archive to use instead of downloading
  --download-only        Download the archive but do not extract
  --skip-download        Require an existing archive and skip network fetches
  --force-extract        Extract even if verification passes already
  --min-free-gb N        Minimum free disk space required on the target volume
  -h, --help             Show this help
EOF
}

log() {
  printf '[restore-assets] %s\n' "$*"
}

fail() {
  printf '[restore-assets] ERROR: %s\n' "$*" >&2
  exit 1
}

require_tool() {
  command -v "$1" >/dev/null 2>&1 || fail "Required tool not found: $1"
}

resolve_hash_for_version() {
  awk -F': ' -v version="$1" '$1 == version { print $2; exit }' "$CONTENT_VERSIONS"
}

free_gb_for_path() {
  df -Pk "$1" | awk 'NR == 2 { printf "%.0f", $4 / 1024 / 1024 }'
}

verify_restored_assets() {
  local missing=0
  local required_paths=(
    "$TARGET_CONTENT_DIR/Static/Pole/SM_Pole09.uasset"
    "$TARGET_CONTENT_DIR/Static/GenericMaterials/Sidewalk/Textures/T_Sidewalk_05_d.uasset"
    "$TARGET_CONTENT_DIR/Static/GenericMaterials/Vehicles/CarPaints/MI_CarPaint_Metallic_Blue01.uasset"
    "$TARGET_CONTENT_DIR/Static/Static/00_LegacyAssets/SM_PlantpotM2.uasset"
    "$TARGET_CONTENT_DIR/Static/Static/00_LegacyAssets/SMF_PlantpotM2.uasset"
    "$TARGET_CONTENT_DIR/Static/Vegetation/Bushes/SMF_Pine_Bush.uasset"
  )

  for path in "${required_paths[@]}"; do
    if [[ ! -f "$path" ]]; then
      printf '[restore-assets] missing after restore: %s\n' "$path" >&2
      missing=1
    fi
  done

  return "$missing"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      shift
      VERSION="${1:-}"
      ;;
    --archive)
      shift
      ARCHIVE_PATH="${1:-}"
      ;;
    --download-only)
      DOWNLOAD_ONLY=true
      ;;
    --skip-download)
      SKIP_DOWNLOAD=true
      ;;
    --force-extract)
      FORCE_EXTRACT=true
      ;;
    --min-free-gb)
      shift
      MIN_FREE_GB="${1:-}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
  shift
done

require_tool curl
require_tool tar
require_tool awk

[[ -f "$CONTENT_VERSIONS" ]] || fail "Missing content manifest: $CONTENT_VERSIONS"
[[ -d "$TARGET_CONTENT_DIR" ]] || fail "Missing target content directory: $TARGET_CONTENT_DIR"

content_hash="$(resolve_hash_for_version "$VERSION")"
[[ -n "$content_hash" ]] || fail "Could not resolve version '$VERSION' in $CONTENT_VERSIONS"

archive_url="$ASSET_BASE_URL/$content_hash.tar.gz"

if [[ -z "$ARCHIVE_PATH" ]]; then
  mkdir -p "$IMPORT_CACHE_DIR"
  ARCHIVE_PATH="$IMPORT_CACHE_DIR/carla-content-$content_hash.tar.gz"
fi

log "version: $VERSION"
log "content hash: $content_hash"
log "archive url: $archive_url"
log "archive path: $ARCHIVE_PATH"
log "target content dir: $TARGET_CONTENT_DIR"

available_gb="$(free_gb_for_path "$TARGET_CONTENT_DIR")"
if [[ "$available_gb" -lt "$MIN_FREE_GB" ]]; then
  fail "Only ${available_gb}GB free on target volume; require at least ${MIN_FREE_GB}GB"
fi
log "free space check passed: ${available_gb}GB available"

if verify_restored_assets && [[ "$FORCE_EXTRACT" != true ]]; then
  log "required assets already present; skipping extraction"
  exit 0
fi

if [[ ! -f "$ARCHIVE_PATH" ]]; then
  if [[ "$SKIP_DOWNLOAD" == true ]]; then
    fail "Archive not found and --skip-download was requested"
  fi

  log "downloading archive with resume support"
  curl -L --fail --continue-at - -o "$ARCHIVE_PATH" "$archive_url"
else
  log "using existing archive"
  if [[ "$SKIP_DOWNLOAD" != true && "$DOWNLOAD_ONLY" != true ]]; then
    log "refreshing archive with resume support in case it is partial"
    curl -L --fail --continue-at - -o "$ARCHIVE_PATH" "$archive_url"
  fi
fi

if [[ "$DOWNLOAD_ONLY" == true ]]; then
  log "download-only mode complete"
  exit 0
fi

log "extracting archive into target content directory"
tar -xzf "$ARCHIVE_PATH" -C "$TARGET_CONTENT_DIR"

if verify_restored_assets; then
  log "asset verification passed"
else
  fail "asset verification failed after extraction"
fi

log "restore complete"