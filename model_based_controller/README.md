# Model Based Controller

This is a ros2-control model based controller for quadruped robots.
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
cd ~/colcon_ws
colcon build --packages-up-to model_based_controller
```

## 3. Launch

### 3.1 Mujoco Simulation
```bash
source ~/colcon_ws/install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py pkg_description:=arcdog_description
```