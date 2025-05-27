#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # 1) New launch argument: choose hardware vs built-in JSP
    declare_use_hw_jsp = DeclareLaunchArgument(
        'use_hw_jsp',
        default_value='true',
        description='Use hardware joint_state_publisher instead of the default one'
    )

    # reuse existing args
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz', default_value='false', description='Enable RViz2'
    )
    declare_enable_jsp_gui = DeclareLaunchArgument(
        'enable_jsp_gui', default_value='false', description='Enable Joint State Publisher GUI'
    )
    declare_enable_foxglove_bridge = DeclareLaunchArgument(
        'enable_foxglove_bridge', default_value='true', description='Enable Foxglove Bridge'
    )

    # 2) Read configurations
    use_hw_jsp = LaunchConfiguration('use_hw_jsp')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_jsp_gui = LaunchConfiguration('enable_jsp_gui')
    enable_foxglove_bridge = LaunchConfiguration('enable_foxglove_bridge')

    # 3) Load URDF
    pkg_share = get_package_share_directory('mini_bdx_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'mini_bdx.urdf')
    with open(urdf_path, 'r') as inf:
        robot_description = inf.read()

    # 4) Default JSP (runs only if use_hw_jsp is false)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_hw_jsp)
    )

    # 5) Robot State Publisher (always)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 6) Hardware JSP (runs only if use_hw_jsp is true)
    hw_jsp = Node(
        package='mini_bdx_description',
        executable='mini_bdx_joint_state_publisher.py',
        name='mini_bdx_joint_state_publisher',
        output='screen',
        condition=IfCondition(use_hw_jsp)
    )

    # 7) Optional GUI slider
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(enable_jsp_gui)
    )

    # 8) Optional RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz)
    )

    # 9) Optional Foxglove Bridge
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch',
            'foxglove_bridge_launch.xml'
        )),
        condition=IfCondition(enable_foxglove_bridge)
    )

    return LaunchDescription([
        declare_use_hw_jsp,
        declare_enable_rviz,
        declare_enable_jsp_gui,
        declare_enable_foxglove_bridge,
        jsp,
        rsp,
        hw_jsp,
        jsp_gui,
        rviz,
        foxglove_bridge,
    ])
