# Mujoco Simulator

This is a ros2 packages of robot simulation based on Mujoco

Tested environment:

* Ubuntu 22.04
    * ROS2 Humble

# 1 Dependency

1. mujoco

# 2 Installation

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

# 3 Run Example
1. Open a terminal to launch the mujoco emulator
```
ros2 launch mujoco_simulator mujoco.launch.py
```
