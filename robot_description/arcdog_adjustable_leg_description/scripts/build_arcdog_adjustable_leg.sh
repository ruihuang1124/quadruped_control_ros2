#!/bin/bash

set -e

robot=arcdog_adjustable_leg
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "${script_dir}/.." && pwd)"

build_urdf() {

  xacro_dir="${package_dir}/xacro"
  urdf_dir="${package_dir}/urdf"

  if [ -d "$xacro_dir" ]; then
    echo "==== Building: $robot | Simulator: $1 ===="

    if [ "$1" == "none" ]; then
      xacro "$xacro_dir/robot.xacro" simulator:="$1" >"$urdf_dir/$robot.urdf"
      check_urdf "$urdf_dir/$robot.urdf"
    else
      xacro "$xacro_dir/robot.xacro" simulator:="$1" >"$urdf_dir/${robot}_$1.urdf"
      check_urdf "$urdf_dir/${robot}_$1.urdf"
    fi

  else
    echo "ERROR: The xacro directory $xacro_dir does not exist."
    exit 1
  fi
}

# rm -f "$PWD"/../urdf/quadrupedal_robot/$robot/*.urdf
# rm -f "$PWD"/../urdf/quadrupedal_robot/$robot/arcdog_test.urdf
# mkdir -p "$PWD"/../urdf/quadrupedal_robot/$robot

#build_urdf ocs2
# build_urdf test
# build_urdf mujoco

# 检查是否传入了参数，如果没有传入，给一个默认值（比如 test）或者报错退出
if [ -z "${1:-}" ]; then
  echo "Usage: ./build.sh <simulator_name>"
  echo "Example: ./build.sh ocs2"
  exit 1
fi

# 将终端传入的第一个参数 $1 传递给 build_urdf 函数
build_urdf "$1"

exit 0
