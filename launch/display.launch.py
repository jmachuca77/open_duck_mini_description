#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
# ← fixed import below:
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # launch arguments
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz', default_value='false', description='Enable RViz2'
    )
    declare_enable_jsp_gui = DeclareLaunchArgument(
        'enable_jsp_gui', default_value='false', description='Enable Joint State Publisher GUI'
    )
    declare_enable_foxglove_bridge = DeclareLaunchArgument(
        'enable_foxglove_bridge', default_value='true', description='Enable Foxglove WebSocket Bridge'
    )

    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_jsp_gui = LaunchConfiguration('enable_jsp_gui')
    enable_foxglove_bridge = LaunchConfiguration('enable_foxglove_bridge')

    # load URDF
    pkg_share = get_package_share_directory('mini_bdx_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'mini_bdx.urdf')
    with open(urdf_path, 'r') as inf:
        robot_description = inf.read()

    # core nodes
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # conditional GUI
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(enable_jsp_gui)
    )

    # conditional RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(enable_rviz)
    )

    # conditional Foxglove bridge
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch',
            'foxglove_bridge_launch.xml'
        )),
        condition=IfCondition(enable_foxglove_bridge)
    )

    return LaunchDescription([
        declare_enable_rviz,
        declare_enable_jsp_gui,
        declare_enable_foxglove_bridge,
        jsp,
        jsp_gui,
        rsp,
        rviz,
        foxglove_bridge,
    ])
