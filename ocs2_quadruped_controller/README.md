# OCS2 Quadruped Controller

This is a ros2-control controller based on [legged_control](https://github.com/qiayuanl/legged_control)
and [ocs2_ros2](https://github.com/legubiao/ocs2_ros2).

Tested environment:

* Ubuntu 22.04
    * ROS2 Humble

# 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
    * joint velocity
    * joint effort
    * KP
    * KD
* state:
    * joint effort
    * joint position
    * joint velocity
    * imu sensor
        * linear acceleration
        * angular velocity
        * orientation
    * feet force sensor

# 2 Dependency

1. qpOASES
2. ros2_ocs2
3. mujoco

# 3 Installation

## 3.1 Install Dependency

### 3.1.1 qpOASES

```
git clone https://github.com/coin-or/qpOASES.git
mkdir build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```
option `-DCMAKE_POSITION_INDEPENDENT_CODE=ON` is necessary. If you already installed qpOASES without this option, you need to reinstall it


### 3.1.2 ros2_ocs2

[Installation](https://github.com/ruihuang1124/ros2_ocs2)

### 3.1.3 mujoco

mujoco should be installed by building from source with release version 3.2.7.

1. Clone the mujoco repository:
```
git clone https://github.com/deepmind/mujoco.git
```
2. cd in mujoco path, reate a new build directory and cd into it:
```
cd mujoco
mkdir build && cd build
```
3. Configure the build, build and install
```
cmake ..
make
sudo make install
```

## 3.2 Install Project

1. Clone repository:
```
mkdir -p colcon_ws/src
cd src
git clone https://github.com/ruihuang1124/quadruped_control_ros2.git
```
2. Cd in ros workspace and build
```
cd colcon_ws
colcon build --packages-up-to cmd_mapping custom_msgs hardware_mujoco hardware_arcdog hardware_mujoco_piper keyboard_input leg_pd_controller mujoco_simulator arcdog_description ocs2_quadruped_controller unitree_guide_controller piper_with_gripper_moveit piper_description sirius_description 
```

# 4 Run Example
1. Open a terminal to launch the mujoco emulator
```
ros2 launch mujoco_simulator mujoco.launch.py
```
2. Open a terminal to run controller
```
ros2 launch ocs2_quadruped_controller controller.launch.py
```
3. Enter the control command in the terminal that receives the keyboard command. You should input `9` first
```
r,f : control body height
1   : stance gait
2   : trot gait
3   : walk trot gait
4   : fly trot gait
w,a,b,d : linear velocity control
j,l : yaw velocity control
9   : activate the controller
```

At the first launch, controller may compile the OCS2 model and generate the shared library. The compilation process may take a few minutes. After the compilation, restart the controller and the robot should stand up. Then you can use the keyboard or joystick to control the robot (Keyboard 2 or Joystick LB+A to Trot mode).