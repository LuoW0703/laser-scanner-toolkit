#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-"$PROJECT_DIR/build"}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

cmake_args=(
  -S "$PROJECT_DIR"
  -B "$BUILD_DIR"
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
)

if [[ -n "${OpenCV_DIR:-}" ]]; then
  cmake_args+=("-DOpenCV_DIR=$OpenCV_DIR")
fi

cmake "${cmake_args[@]}"
cmake --build "$BUILD_DIR" --config "$BUILD_TYPE"

echo
echo "Build completed: $BUILD_DIR"
