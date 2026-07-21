import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition  # 新增导入
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    use_gui = LaunchConfiguration('use_gui')  # 获取是否使用 GUI 的参数
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control_E7700.yaml",
        ]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "visualize_urdf.rviz",
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    # ================= 模式 1: GUI 调试模式 =================
    # 只有当 use_gui 为 'true' 时才运行

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)  # 条件启动
    )

    # ================= 模式 2: ros2_control 控制模式 =================
    # 只有当 use_gui 为 'false' (默认) 时才运行
    # 注意：我给下面所有的控制器相关节点加了 condition=UnlessCondition(use_gui)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
        condition=UnlessCondition(use_gui)  # 如果不开GUI，才开控制器
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gui)
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gui)
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rl_quadruped_adjustable_leg_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_gui)
    )

    cmd_mapping = Node(
        package="cmd_mapping",
        executable="cmd_mapping",
        condition=UnlessCondition(use_gui)
    )

    joy_node = Node(
        package='joy',
        namespace='',
        executable='game_controller_node',
        name='joy_package',
        condition=UnlessCondition(use_gui)
    )

    # ================= 返回节点列表 =================

    nodes_to_start = [
        joy_node,
        rviz,
        cmd_mapping,
        robot_state_publisher,
        joint_state_publisher_gui_node,  # 新增的 GUI 节点
        controller_manager,
        joint_state_publisher,
    ]

    # 事件处理器也需要根据条件判断，或者简单地让它们依赖于 spawner
    # 如果 spawner 不启动，这些 handler 也就不会触发，所以直接放这里没问题
    nodes_to_start.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        )
    )

    nodes_to_start.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[controller],
            )
        )
    )

    return nodes_to_start


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='arcdog_adjustable_leg_description',
        description='package for robot description'
    )

    # 新增参数定义
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description=(
            'If true, launch joint_state_publisher_gui and disable ros2_control'
        )
    )
    return LaunchDescription([
        pkg_description,
        use_gui_arg,  # 注册参数
        OpaqueFunction(function=launch_setup),
    ])
