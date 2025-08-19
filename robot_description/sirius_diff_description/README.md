# Sirius Description

This repository contains the urdf model of go2.

![go2](../../../.images/go2.png)

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

## Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to sirius_description --symlink-install
```

## Visualize the robot

To visualize and check the configuration of the robot in rviz, simply launch:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch sirius_description visualize.launch.py
```


