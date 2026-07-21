from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_pkg = DeclareLaunchArgument(
        "robot_pkg",
        default_value="arcdog_adjustable_leg_description",
        description="Package containing the adjustable-leg MuJoCo scene",
    )
    max_speed = DeclareLaunchArgument(
        "box_response_max_speed_mps",
        default_value="0.080",
        description="Maximum rate of each applied box position target in metres per second",
    )

    robot_pkg_name = LaunchConfiguration("robot_pkg")
    xml_file_path = PathJoinSubstitution(
        [FindPackageShare(robot_pkg_name), "xml", "scene.xml"]
    )

    mujoco_node = Node(
        package="mujoco_simulator",
        executable="mujoco_simulator",
        output="screen",
        parameters=[
            {
                "xml_file_path": xml_file_path,
                "box_response_enabled": True,
                "box_response_max_speed_mps": ParameterValue(
                    LaunchConfiguration("box_response_max_speed_mps"), value_type=float
                ),
                "box_target_lower_m": 0.0,
                "box_target_upper_m": 0.060,
            }
        ],
    )

    return LaunchDescription([robot_pkg, max_speed, mujoco_node])
