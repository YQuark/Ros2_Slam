#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/../.."

SDK_SRC="${SDK_SRC:-vendor/ydlidar-sdk}"
SDK_BUILD="${SDK_BUILD:-/tmp/slamrobot-build/ydlidar-sdk}"
SDK_PREFIX="${SDK_PREFIX:-/opt/slamrobot/vendor/ydlidar-sdk}"

if [ ! -d "$SDK_SRC" ]; then
    echo "Missing $SDK_SRC. Run ./scripts/bootstrap/fetch_vendor.sh first." >&2
    exit 1
fi

cmake -S "$SDK_SRC" -B "$SDK_BUILD" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TEST=OFF \
    -DBUILD_CSHARP=OFF
cmake --build "$SDK_BUILD" --parallel 2
cmake --install "$SDK_BUILD"
