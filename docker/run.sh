#!/usr/bin/env bash
# Starts (or re-attaches to) the development container with the repo bind-mounted
# and port 3000 published for the webpack dev server.
#
#   docker/run.sh                 -> interactive bash
#   docker/run.sh scripts/build_all.sh   -> run a command and exit
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="modern-robotics-wasm:dev"
CONTAINER="modern-robotics-wasm"
WORKDIR="/home/robotics/modern-robotics-wasm"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "Image ${IMAGE} not found. Run docker/build.sh first." >&2
    exit 1
fi

if [ "$(docker ps -q -f "name=^${CONTAINER}$")" ]; then
    echo "Attaching to the running ${CONTAINER} container..."
    exec docker exec -it "${CONTAINER}" "${@:-/bin/bash}"
fi

docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true

exec docker run -it --rm \
    --name "${CONTAINER}" \
    -v "${REPO_ROOT}:${WORKDIR}" \
    -w "${WORKDIR}" \
    -p 3000:3000 \
    "${IMAGE}" \
    "${@:-/bin/bash}"
