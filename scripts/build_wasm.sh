#!/usr/bin/env bash
# Compiles the kinematics library and its embind bindings to WebAssembly, dropping
# kinematics.js / kinematics.wasm into web/public/wasm.
# Run this inside the dev container: docker/run.sh scripts/build_wasm.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build/wasm"

if ! command -v emcmake >/dev/null 2>&1; then
    echo "emcmake not found. Run this inside the dev container (docker/run.sh)." >&2
    exit 1
fi

emcmake cmake -S "${REPO_ROOT}/cpp" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j "$(nproc)"

echo
echo "WebAssembly module written to web/public/wasm:"
ls -la "${REPO_ROOT}/web/public/wasm"
