#!/usr/bin/env bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")"; pwd)
# THIRDPARTY_ROOT="${THIRDPARTY_ROOT:-/home/kangneng/xq/code/thirdparty/pct-install}"
THIRDPARTY_ROOT="${THIRDPARTY_ROOT:-/home/code/thirdparty/pct-install}"


# GTSAM
GTSAM_SRC="${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1"
GTSAM_INSTALL="${THIRDPARTY_ROOT}/gtsam-4.1.1"

rm -rf "${GTSAM_SRC}/build" "${GTSAM_SRC}/install" "${GTSAM_INSTALL}"
mkdir -p "${GTSAM_SRC}/build" "${GTSAM_INSTALL}"

cmake -S "${GTSAM_SRC}" -B "${GTSAM_SRC}/build" \
  -DCMAKE_INSTALL_PREFIX="${GTSAM_INSTALL}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_SYSTEM_EIGEN=ON
cmake --build "${GTSAM_SRC}/build" -j6
cmake --install "${GTSAM_SRC}/build"

# OSQP
OSQP_SRC="${ROOT_DIR}/lib/3rdparty/osqp"
OSQP_INSTALL="${THIRDPARTY_ROOT}/osqp"

rm -rf "${OSQP_SRC}/build" "${OSQP_SRC}/install" "${OSQP_INSTALL}"
mkdir -p "${OSQP_SRC}/build" "${OSQP_INSTALL}"

cmake -S "${OSQP_SRC}" -B "${OSQP_SRC}/build" \
  -DCMAKE_INSTALL_PREFIX="${OSQP_INSTALL}" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${OSQP_SRC}/build" -j4
cmake --install "${OSQP_SRC}/build"
