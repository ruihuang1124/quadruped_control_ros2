#!/bin/bash

sudo -E bash -c '
source /opt/ros/humble/setup.bash;
source /home/ray/colcon_ws/install/setup.bash;
export LD_LIBRARY_PATH="/home/ray/software/libtorch/lib:/opt/ros/humble/lib:/home/ray/colcon_ws/install/lib:$LD_LIBRARY_PATH";
echo "=== LD_LIBRARY_PATH ===";
echo "$LD_LIBRARY_PATH";
ros2 launch rl_quadruped_controller mujoco.launch.py
'