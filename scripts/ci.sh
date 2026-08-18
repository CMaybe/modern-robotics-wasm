#!/usr/bin/env bash
# The full check suite, exactly as CI runs it.
#
# CI is deliberately thin: the workflow builds the dev image and calls this
# script, so `docker/run.sh scripts/ci.sh` reproduces a CI run locally.
#
# Deliberately excluded: the vendor mesh download (a third-party network
# dependency) and the browser tests. Those live in the separate `e2e` job.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

step() { printf '\n\033[1m==> %s\033[0m\n' "$1"; }

step "clang-format"
mapfile -t sources < <(find cpp -name '*.cpp' -o -name '*.hpp' -o -name '*.inl' | sort)
unformatted=()
for file in "${sources[@]}"; do
    clang-format --dry-run --Werror "${file}" >/dev/null 2>&1 || unformatted+=("${file}")
done
if [ ${#unformatted[@]} -gt 0 ]; then
    printf 'These files do not match .clang-format:\n'
    printf '  %s\n' "${unformatted[@]}"
    printf 'Fix with: clang-format -i %s\n' "${unformatted[*]}"
    exit 1
fi
echo "${#sources[@]} files clean"

step "Native build and unit tests"
scripts/build_native.sh
ctest --test-dir build/native --output-on-failure

step "WebAssembly build"
scripts/build_wasm.sh

step "WebAssembly binding smoke test"
node scripts/smoke_wasm.cjs

step "Frontend dependencies"
cd web
if [ -f package-lock.json ]; then
    npm ci --no-audit --no-fund
else
    npm install --no-audit --no-fund
fi

step "Frontend type check"
npx tsc --noEmit

step "Frontend production build"
npx webpack --mode production

printf '\n\033[1;32mAll CI checks passed.\033[0m\n'
