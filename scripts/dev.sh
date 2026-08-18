#!/usr/bin/env bash
# One-shot developer entry point: build the WASM module, install web deps if needed,
# then serve the frontend on http://localhost:3000.
# Run this inside the dev container: docker/run.sh scripts/dev.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

"${REPO_ROOT}/scripts/build_wasm.sh"

# The vendor meshes are a third-party download, so fetch them once on first run.
# Without them the viewer still works, falling back to its schematic rendering.
if [ ! -d "${REPO_ROOT}/web/public/robots" ]; then
    echo
    echo "Robot meshes not found — fetching them (~32 MB)..."
    "${REPO_ROOT}/scripts/fetch_meshes.sh"
fi

cd "${REPO_ROOT}/web"
if [ ! -d node_modules ]; then
    echo "Installing frontend dependencies..."
    npm install
fi

echo
echo "Starting the dev server on http://localhost:3000"
npm start
