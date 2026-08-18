#!/usr/bin/env bash
# Builds the development image, mapping the host UID/GID so files created inside
# the container stay editable on the host.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t modern-robotics-wasm:dev \
    -f "${REPO_ROOT}/docker/Dockerfile" \
    "${REPO_ROOT}"

echo
echo "Built modern-robotics-wasm:dev — start it with docker/run.sh"
