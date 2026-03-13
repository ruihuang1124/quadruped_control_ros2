#!/bin/bash
# 16-DOF 弹簧腿机器狗 - ocs2_2 编译脚本
set -e

WS_DIR="/home/mallow/ocs2_2"
UPSTREAM_WS="/home/mallow/control_1/quadruped_control_ros2"

echo "================================================"
echo "  编译 16-DOF 弹簧腿机器狗 OCS2 控制器"
echo "================================================"

# 1. 加载 ROS2 基础环境
echo "[1/4] 加载环境..."
source /opt/ros/humble/setup.bash

# 2. 加载上游OCS2依赖工作空间
if [ -f "${UPSTREAM_WS}/install/setup.bash" ]; then
    echo "  -> 加载上游工作空间: ${UPSTREAM_WS}"
    source "${UPSTREAM_WS}/install/setup.bash"
elif [ -f "${UPSTREAM_WS}/setup.bash" ]; then
    echo "  -> 加载上游工作空间: ${UPSTREAM_WS}"
    source "${UPSTREAM_WS}/setup.bash"
else
    echo "  警告: 未找到上游工作空间，尝试继续..."
fi

# 3. 进入工作空间
cd "${WS_DIR}"

# 4. 编译 (ocs2_2 作为 colcon 工作空间，其内部包直接列出)
echo "[2/4] 运行 colcon build..."

# 清理Anaconda的Python干扰
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:$PATH"
unset CONDA_PREFIX CONDA_DEFAULT_ENV

export MAKEFLAGS="-j2"
colcon build \
    --executor sequential \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPython3_EXECUTABLE=/usr/bin/python3 \
    --packages-select \
        ocs2_quadruped_controller \
        arcdog_adjustable_leg_description \
        keyboard_input \
        hardware_mujoco \
    2>&1

BUILD_EXIT=$?
if [ $BUILD_EXIT -ne 0 ]; then
    echo ""
    echo "编译失败! 退出码: $BUILD_EXIT"
    echo "查看上面的错误信息"
    exit $BUILD_EXIT
fi

echo "[3/4] 加载编译结果..."
source "${WS_DIR}/install/setup.bash"

echo "[4/4] 编译完成!"
echo ""
echo "================================================"
echo "  启动方法:"
echo "================================================"
echo ""
echo "方法1 - 一键启动 (MuJoCo + OCS2 + 键盘控制):"
echo "  ./launch_all.sh"
echo ""
echo "方法2 - 手动分步启动:"
echo "  步骤1: source ./install/setup.bash"
echo "  步骤2: ros2 launch arcdog_adjustable_leg_description controller.launch.py"
echo "  步骤3: (另开终端) ros2 control switch_controllers --activate ocs2_quadruped_controller"
echo "  步骤4: (另开终端) ros2 run keyboard_input keyboard_publisher"
echo ""
