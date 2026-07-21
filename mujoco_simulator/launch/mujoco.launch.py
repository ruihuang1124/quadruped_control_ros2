from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 设置参数的默认值和描述
    robot_pkg = DeclareLaunchArgument(
        "robot_pkg",
        default_value="arcdog_adjustable_leg_description",
        description="package for robot description",
    )

    # 读取参数
    robot_pkg_name = LaunchConfiguration("robot_pkg")
    contract_diagnostic = LaunchConfiguration("highstep_contract_diagnostic")
    contract_platform_height = LaunchConfiguration("highstep_contract_platform_height_m")

    # 根据参数找到xml文件
    xml_file_path = PathJoinSubstitution(
        [
            FindPackageShare(robot_pkg_name),  # 包路径
            "xml",
            "scene.xml",
        ]
    )

    # 加载mujoco node
    mujoco_node = Node(
        package="mujoco_simulator",
        executable="mujoco_simulator",
        output="screen",
        parameters=[{
            "xml_file_path": xml_file_path,
            "highstep_contract_diagnostic_enabled": ParameterValue(
                contract_diagnostic, value_type=bool
            ),
            "highstep_contract_platform_height_m": ParameterValue(
                contract_platform_height, value_type=float
            ),
        }],
    )

    contract_diagnostic_arg = DeclareLaunchArgument(
        "highstep_contract_diagnostic",
        default_value="false",
        description=(
            "Arm the explicit B300 fixed-motion MuJoCo contract diagnostic interface"
        ),
    )
    contract_platform_height_arg = DeclareLaunchArgument(
        "highstep_contract_platform_height_m",
        default_value="0.30",
        description=(
            "Physical MuJoCo platform height; never provided to the policy adapter"
        ),
    )

    return LaunchDescription(
        [
            robot_pkg,
            contract_diagnostic_arg,
            contract_platform_height_arg,
            mujoco_node,
        ]
    )
