#!/bin/bash
# 16-DOF 弹簧腿机器狗一键启动脚本

WS_DIR="/home/mallow/ocs2_2"
UPSTREAM_WS="/home/mallow/control_1/quadruped_control_ros2"

echo "清理可能残留的后台进程..."
killall -9 ros2_control_node spawner mujoco_simulator robot_state_publisher keyboard_publisher cmd_mapping 2>/dev/null || true
echo "加载环境..."
source /opt/ros/humble/setup.bash
source "${UPSTREAM_WS}/install/setup.bash" 2>/dev/null || true
source "${WS_DIR}/install/setup.bash"

echo "[1/4] 启动 MuJoCo 仿真 + OCS2 控制器..."
gnome-terminal --title="MuJoCo + OCS2" -- bash -c "
source /opt/ros/humble/setup.bash
source ${UPSTREAM_WS}/install/setup.bash 2>/dev/null
source ${WS_DIR}/install/setup.bash
echo '>>> 正在启动 MuJoCo 仿真环境...'
ros2 launch ocs2_quadruped_controller mujoco_arcdog.launch.py &
MUJOCO_PID=\$!
sleep 8
echo '>>> 正在启动 OCS2 控制器节点...'
ros2 launch ocs2_quadruped_controller controller.launch.py
wait \$MUJOCO_PID
exec bash" &

# 等待 controller_manager 的 configure 服务就绪
echo ">>> 等待 OCS2 控制器加载 (可能需要1-2分钟)..."
TIMEOUT=300
ELAPSED=0
while true; do
    if ros2 service list 2>/dev/null | grep -q /controller_manager/configure_controller; then
        echo ">>> 控制器管理器服务已出现，等待 OCS2 回调启动..."
        break
    fi
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo ">>> 错误: 等待超时 (${TIMEOUT}s)，控制器未启动!"
        exit 1
    fi
    sleep 3
    ELAPSED=$((ELAPSED + 3))
done

# 额外等待确保 ocs2_quadruped_controller 已被 spawner 彻底加载 (CppAD 编译可能长达50秒)
echo ">>> 首次启动需要通过 CppAD 编译底层动力学库，请耐心等待 (约30~60秒)..."
sleep 45

echo "[2/4] 配置控制器 (unconfigured -> inactive)..."
for i in {1..15}; do
    result=$(ros2 service call /controller_manager/configure_controller \
        controller_manager_msgs/srv/ConfigureController \
        "{name: 'ocs2_quadruped_controller'}" 2>/dev/null)
    if echo "$result" | grep -q "ok=True"; then
        echo ">>> 控制器配置成功!"
        break
    else
        echo ">>> 等待 CppAD 解析 ($i/15) ，重试中..."
        sleep 5
    fi
done

sleep 2

echo "[3/4] 激活控制器 (inactive -> active)..."
ros2 control switch_controllers --activate ocs2_quadruped_controller
if [ $? -eq 0 ]; then
    echo ">>> 控制器激活成功!"
else
    echo ">>> 激活失败！可手动执行:"
    echo "    ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController \"{name: 'ocs2_quadruped_controller'}\""
    echo "    ros2 control switch_controllers --activate ocs2_quadruped_controller"
fi

echo "[4/4] 启动键盘控制..."
gnome-terminal --title="键盘控制 WASD=移动 QE=转向 1-8=步态" -- bash -c "
source /opt/ros/humble/setup.bash
source ${UPSTREAM_WS}/install/setup.bash 2>/dev/null
source ${WS_DIR}/install/setup.bash
echo '================================================'
echo '  WASD=移动  QE=转向  空格=停止'
echo '  1=站立 2=trot 3=walk 4=dynamic_walk'
echo '  5=pace 6=bound 7=standing_trot 8=amble'
echo '  +/-=调高度  P=被动模式'
echo '  提示：先按 2 (trot步态) 再用 WASD 移动！'
echo '================================================'
ros2 run cmd_mapping cmd_mapping &
ros2 run keyboard_input keyboard_publisher
exec bash" &

echo ""
echo "=== 启动完成! ==="
echo "请在键盘控制终端中按 [2] 切换到 trot 步态，然后用 WASD 控制机器狗移动。"
