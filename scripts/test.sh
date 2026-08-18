#!/usr/bin/env bash
# Builds and runs the native GoogleTest suite.
# Run this inside the dev container: docker/run.sh scripts/test.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build/native"

"${REPO_ROOT}/scripts/build_native.sh"

ctest --test-dir "${BUILD_DIR}" --output-on-failure
