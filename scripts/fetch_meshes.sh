#!/usr/bin/env bash
# Downloads the official robot descriptions and their visual meshes, converts the
# xacro sources into plain URDFs, and stages everything under web/public/robots/.
#
# The assets are third-party and large (~32 MB), so they are fetched on demand
# rather than committed. Both upstreams are permissively licensed:
#   * Universal Robots description — BSD-3-Clause
#   * Franka Emika description     — Apache-2.0
# Their LICENSE files are copied alongside the meshes.
#
# Run this inside the dev container: docker/run.sh scripts/fetch_meshes.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${REPO_ROOT}/web/public/robots"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT

UR_REPO="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git"
FRANKA_REPO="https://github.com/frankaemika/franka_description.git"

echo "Installing xacro..."
pip3 install --quiet --user xacro
export PATH="${HOME}/.local/bin:${PATH}"

# ROS 2 xacro resolves $(find <pkg>) through ament_index_python, which only exists
# inside a ROS install. This stub resolves package names against the clone dir.
mkdir -p "${WORK_DIR}/stub/ament_index_python"
cat > "${WORK_DIR}/stub/ament_index_python/__init__.py" <<'PYTHON'
import os


def get_package_share_directory(name):
    root = os.environ["XACRO_PKG_ROOT"]
    path = os.path.join(root, name)
    if not os.path.isdir(path):
        raise KeyError(name)
    return path


def get_package_prefix(name):
    return get_package_share_directory(name)
PYTHON
cp "${WORK_DIR}/stub/ament_index_python/__init__.py" "${WORK_DIR}/stub/ament_index_python/packages.py"
export PYTHONPATH="${WORK_DIR}/stub"

PKG_ROOT="${WORK_DIR}/pkgs"
mkdir -p "${PKG_ROOT}"
export XACRO_PKG_ROOT="${PKG_ROOT}"

echo "Cloning Universal Robots description..."
git clone --depth 1 --quiet "${UR_REPO}" "${PKG_ROOT}/ur_description"
echo "Cloning Franka description..."
git clone --depth 1 --quiet "${FRANKA_REPO}" "${PKG_ROOT}/franka_description"

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}/ur5" "${OUT_DIR}/fr3"

echo "Generating ur5.urdf..."
xacro "${PKG_ROOT}/ur_description/urdf/ur.urdf.xacro" ur_type:=ur5 name:=ur5 > "${OUT_DIR}/ur5/ur5.urdf"

# hand:=false keeps the flange as the tip, matching the kinematics model in cpp/.
echo "Generating fr3.urdf..."
xacro "${PKG_ROOT}/franka_description/robots/fr3/fr3.urdf.xacro" hand:=false > "${OUT_DIR}/fr3/fr3.urdf"

# Only the visual meshes are copied: urdf-loader is configured not to parse
# collision geometry, and the collision STLs would double the download.
echo "Copying visual meshes..."
mkdir -p "${OUT_DIR}/ur5/meshes/ur5/visual"
cp "${PKG_ROOT}"/ur_description/meshes/ur5/visual/*.dae "${OUT_DIR}/ur5/meshes/ur5/visual/"

mkdir -p "${OUT_DIR}/fr3/meshes/robots/fr3/visual"
cp "${PKG_ROOT}"/franka_description/meshes/robots/fr3/visual/*.dae "${OUT_DIR}/fr3/meshes/robots/fr3/visual/"

echo "Copying licences..."
cp "${PKG_ROOT}/ur_description/LICENSE" "${OUT_DIR}/ur5/LICENSE"
cp "${PKG_ROOT}/franka_description/LICENSE" "${OUT_DIR}/fr3/LICENSE"

UR_COMMIT="$(git -C "${PKG_ROOT}/ur_description" rev-parse --short HEAD)"
FRANKA_COMMIT="$(git -C "${PKG_ROOT}/franka_description" rev-parse --short HEAD)"

cat > "${OUT_DIR}/ATTRIBUTION.md" <<ATTRIBUTION
# Third-party robot descriptions

These files are **not** part of this project. They are fetched by
\`scripts/fetch_meshes.sh\` from the robot vendors' official descriptions and are
redistributed here under their own licences.

## ur5/ — Universal Robots UR5

- Source: ${UR_REPO}
- Commit: ${UR_COMMIT}
- Licence: BSD-3-Clause (see \`ur5/LICENSE\`)
- \`ur5.urdf\` generated with: \`xacro urdf/ur.urdf.xacro ur_type:=ur5 name:=ur5\`

## fr3/ — Franka Research 3

- Source: ${FRANKA_REPO}
- Commit: ${FRANKA_COMMIT}
- Licence: Apache-2.0 (see \`fr3/LICENSE\`)
- \`fr3.urdf\` generated with: \`xacro robots/fr3/fr3.urdf.xacro hand:=false\`

The \`hand:=false\` argument keeps the flange as the end-effector so the rendered
mesh chain ends where the kinematics model in \`cpp/\` does.
ATTRIBUTION

echo
echo "Robot assets staged in web/public/robots:"
du -sh "${OUT_DIR}"/* | sed 's/^/  /'
echo
echo "  ur5 meshes: $(ls -1 "${OUT_DIR}"/ur5/meshes/ur5/visual/*.dae | wc -l)"
echo "  fr3 meshes: $(ls -1 "${OUT_DIR}"/fr3/meshes/robots/fr3/visual/*.dae | wc -l)"
