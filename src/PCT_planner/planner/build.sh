#!/usr/bin/env bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")"; pwd)
# unset THIRDPARTY_ROOT
# THIRDPARTY_ROOT="${THIRDPARTY_ROOT:-/home/kangneng/xq/code/thirdparty/pct-install}"
THIRDPARTY_ROOT="${THIRDPARTY_ROOT:-/home/code/thirdparty/pct-install}"
echo "ROOT_DIR: ${ROOT_DIR}"

cd lib

rm -rf build
mkdir -p build

cd build
cmake ../ \
  -DCMAKE_BUILD_TYPE=Release \
  -DTHIRDPARTY_ROOT="${THIRDPARTY_ROOT}"
make -j6
cp ./src/a_star/a_star*.so ../
cp ./src/trajectory_optimization/traj_opt*.so ../
cp ./src/ele_planner/ele_planner*.so ../
cp ./src/map_manager/py_map_manager*.so ../
cp ./src/common/smoothing/libcommon_smoothing.so ../
cp -L "${THIRDPARTY_ROOT}/gtsam-4.1.1/lib/libmetis-gtsam.so" ../
cp -L "${THIRDPARTY_ROOT}/gtsam-4.1.1/lib/libgtsam.so.4" ../
cd ..

# # optional
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${THIRDPARTY_ROOT}/gtsam-4.1.1/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ROOT_DIR}/lib/build/src/common/smoothing
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/lib
# pybind11-stubgen -o ./ a_star
# pybind11-stubgen -o ./ traj_opt
# pybind11-stubgen -o ./ ele_planner
# pybind11-stubgen -o ./ py_map_manager
# cp ./a_star-stubs/__init__.pyi ./a_star.pyi
# cp ./traj_opt-stubs/__init__.pyi ./traj_opt.pyi
# cp ./ele_planner-stubs/__init__.pyi ./ele_planner.pyi
# cp ./py_map_manager-stubs/__init__.pyi ./py_map_manager.pyi
