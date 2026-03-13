#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_pkg = "arcdog_adjustable_leg_description"
    
    try:
        pkg_path = get_package_share_directory("arcdog_adjustable_leg_description")
        scene_file = os.path.join(pkg_path, "xml", "scene.xml")
        urdf_file = os.path.join(pkg_path, "urdf", "arcdog_adjustable_leg_test.urdf")
        
        print(f"Loading robot from package: {robot_pkg}")
        print(f"Scene file: {scene_file}")
        print(f"URDF file: {urdf_file}")
        
        # 检查文件是否存在
        if not os.path.exists(scene_file):
            print(f"ERROR: Scene file not found: {scene_file}")
            return []
        if not os.path.exists(urdf_file):
            print(f"ERROR: URDF file not found: {urdf_file}")
            return []
        
        mujoco_urdf_file = os.path.join(pkg_path, "urdf", "arcdog_adjustable_leg_mujoco.urdf")

        # MuJoCo仿真器节点 - 使用正确的参数名
        mujoco_simulator = Node(
            package="mujoco_simulator",
            executable="mujoco_simulator",
            name="mujoco_simulator",
            parameters=[
                {
                    "xml_file_path": scene_file,
                    "use_sim_time": True,
                    "publish_rate": 100.0,
                }
            ],
            output="screen",
        )
        
        # 读取URDF文件内容
        robot_description = ""
        with open(urdf_file, 'r') as file:
            robot_description = file.read()
        
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        )

        return [
            mujoco_simulator,
            robot_state_publisher,
        ]
    except Exception as e:
        print(f"Error in launch setup: {e}")
        return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])