#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('curobo_robot_setup').find('curobo_robot_setup')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_setup.rviz')

    # Declare launch arguments
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Path to the URDF file to load'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui if true, otherwise joint_state_publisher'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Path to RViz config file (optional)'
    )

    # Get launch configuration values
    urdf_file = LaunchConfiguration('urdf_file')
    use_gui = LaunchConfiguration('use_gui')
    rviz_config = LaunchConfiguration('rviz_config')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_file,
            'use_sim_time': False
        }],
        arguments=[urdf_file]
    )

    # Joint State Publisher Node (without GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        urdf_file_arg,
        use_gui_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
