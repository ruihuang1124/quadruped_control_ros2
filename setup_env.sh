#!/bin/bash
# 加载 16-DOF 弹簧腿机器狗所有环境（一个install目录包含全部包）

WS_DIR="/home/mallow/ocs2_2"
UPSTREAM_WS="/home/mallow/control_1/quadruped_control_ros2"

source /opt/ros/humble/setup.bash
source "${UPSTREAM_WS}/install/setup.bash" 2>/dev/null || true

# 主工作空间 (包含 ocs2_quadruped_controller, keyboard_input, hardware_mujoco)
source "${WS_DIR}/install/setup.bash"

# arcdog 描述包（单独install目录）
if [ -f "${WS_DIR}/arcdog_adjustable_leg_description/install/setup.bash" ]; then
    source "${WS_DIR}/arcdog_adjustable_leg_description/install/setup.bash"
fi

echo "✓ 16-DOF 弹簧腿机器狗环境加载完成"
