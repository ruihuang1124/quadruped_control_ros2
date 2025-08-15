# RL Quadruped Controller

This repository contains the reinforcement learning based controllers for the quadruped robot.

Tested environment:

* Ubuntu 22.04
    * ROS2 Humble

## 2. Build

### 2.1 Installing libtorch

> You can also choose `libtorch` with cuda. Just remember to download for c++ 11 ABI version. The position to place `libtorch` is also not fixed, just need to config the `.bashrc`.

```bash
cd ~/CLionProjects/
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
```

```bash
cd ~
rm -rf libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip
echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib' >> ~/.bashrc
```

### 2.2 Build Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to rl_quadruped_controller
```

## 3. Launch

### 3.1 Mujoco Simulation
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rl_quadruped_controller mujoco.launch.py pkg_description:=your_robot_description_pkg
```