# Unitree Guide Controller

This is a ros2-control controller based on unitree guide.
Tested environment:

* Ubuntu 22.04
    * ROS2 Humble

## 1. Interfaces

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

## 2. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to unitree_guide_controller
```

## 3. Launch

### 3.1 Mujoco Simulation
> **Warm Reminder**: You need to launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco) before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py pkg_description:=go2_description
```

### 3.2 Gazebo Classic 11 (ROS2 Humble)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go2_description
```

### 3.3 Gazebo Harmonic (ROS2 Jazzy)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_guide_controller gazebo.launch.py pkg_description:=go2_description
```