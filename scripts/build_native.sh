#!/usr/bin/env bash
# Configures and builds the native C++ targets (example + GoogleTest suite).
# Run this inside the dev container: docker/run.sh scripts/build_native.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build/native"
BUILD_TYPE="${BUILD_TYPE:-Release}"

cmake -S "${REPO_ROOT}/cpp" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build "${BUILD_DIR}" -j "$(nproc)"

echo
echo "Built:"
echo "  ${BUILD_DIR}/example/example"
echo "  ${BUILD_DIR}/test/test_kinematics"
