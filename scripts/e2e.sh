#!/usr/bin/env bash
# Browser end-to-end tests: builds the WebAssembly module, installs the frontend
# and Playwright's Chromium, then drives the real page.
#
# The vendor meshes are intentionally not fetched. The viewer falls back to its
# schematic rendering without them, so this suite has no third-party network
# dependency and still covers the WASM-to-WebGL path.
#
# Run this inside the dev container: docker/run.sh scripts/e2e.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

"${REPO_ROOT}/scripts/build_wasm.sh"

cd web
if [ -f package-lock.json ]; then
    npm ci --no-audit --no-fund
else
    npm install --no-audit --no-fund
fi

# Playwright keeps its browsers outside the project so a bind-mounted workspace
# is not polluted, and so CI can cache the directory.
export PLAYWRIGHT_BROWSERS_PATH="${PLAYWRIGHT_BROWSERS_PATH:-${HOME}/.cache/ms-playwright}"
npx playwright install --with-deps chromium

npx playwright test
