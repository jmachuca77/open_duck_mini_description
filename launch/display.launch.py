#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_use_hw_jsp = DeclareLaunchArgument(
        'use_hw_jsp',
        default_value='true',
        description='Use hardware joint_state_publisher instead of the built-in one'
    )
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Enable RViz2'
    )
    declare_enable_jsp_gui = DeclareLaunchArgument(
        'enable_jsp_gui',
        default_value='false',
        description='Enable Joint State Publisher GUI'
    )
    declare_enable_foxglove_bridge = DeclareLaunchArgument(
        'enable_foxglove_bridge',
        default_value='true',
        description='Enable Foxglove WebSocket Bridge'
    )

    # Read configurations
    use_hw_jsp = LaunchConfiguration('use_hw_jsp')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_jsp_gui = LaunchConfiguration('enable_jsp_gui')
    enable_foxglove_bridge = LaunchConfiguration('enable_foxglove_bridge')

    # Load URDF
    pkg_share = get_package_share_directory('open_duck_mini_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'mini_bdx.urdf')
    with open(urdf_path, 'r') as inf:
        robot_description = inf.read()

    # Default JSP (runs only if use_hw_jsp is false)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_hw_jsp)
    )

    # Robot State Publisher (always)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Hardware JSP (runs only if use_hw_jsp is true)
    hw_jsp = Node(
        package='open_duck_mini_description',
        executable='mini_bdx_joint_state_publisher.py',
        name='mini_bdx_joint_state_publisher',
        output='screen',
        condition=IfCondition(use_hw_jsp)
    )

    # BNO055 IMU node
    bno_config = os.path.join(
        get_package_share_directory('open_duck_mini_description'),
        'config',
        'bno055_params_i2c.yaml'
    )
    bno_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno_config]
    )

    # Feet Switch Node (always)
    feet_switch = Node(
        package='open_duck_mini_description',
        executable='feet_switch_node.py',
        name='feet_switch_node',
        output='screen'
    )

    # Walk Policy Inference Node (always)
    walk_policy = Node(
        package='open_duck_mini_description',
        executable='walk_policy_inference_node.py',
        name='walk_policy_inference_node',
        output='screen'
    )

    # Joint State Publisher GUI (optional)
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(enable_jsp_gui)
    )

    # RViz2 (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz)
    )

    # Foxglove Bridge (optional)
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch',
            'foxglove_bridge_launch.xml'
        )),
        condition=IfCondition(enable_foxglove_bridge)
    )

    # Teleop Twist Joy Launch (PS5 config via config_filepath)
    ps5_config_file = os.path.join(
        get_package_share_directory('open_duck_mini_description'),
        'config',
        'ps5.config.yaml'
    )
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('teleop_twist_joy'),
            'launch',
            'teleop-launch.py'
        )),
        launch_arguments={'config_filepath': ps5_config_file}.items()
    )

    return LaunchDescription([
        declare_use_hw_jsp,
        declare_enable_rviz,
        declare_enable_jsp_gui,
        declare_enable_foxglove_bridge,
        jsp,
        rsp,
        hw_jsp,
        bno_node,
        feet_switch,
        walk_policy,
        jsp_gui,
        rviz,
        foxglove_bridge,
        teleop_launch
    ])
