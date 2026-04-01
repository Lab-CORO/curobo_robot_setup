#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def launch_setup(context, *args, **kwargs):
    """Setup function to read URDF file content"""
    # Get package share directory
    pkg_share = FindPackageShare('curobo_robot_setup').find('curobo_robot_setup')

    # Get launch configuration values
    urdf_file_path = LaunchConfiguration('urdf_file').perform(context)
    use_gui = LaunchConfiguration('use_gui')
    rviz_config = LaunchConfiguration('rviz_config')

    # Read URDF file content (Fix Bug #1)
    robot_description_content = ''
    if urdf_file_path and os.path.exists(urdf_file_path):
        with open(urdf_file_path, 'r') as file:
            robot_description_content = file.read()
    else:
        print(f"Warning: URDF file not found or not specified: {urdf_file_path}")

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,  # Fixed: pass content, not path
            'use_sim_time': False
        }]
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
        parameters=[{
            'use_sim_time': False,
            'urdf_file': urdf_file_path,  # forwarded to CuRoboSetupPanel for auto-load
        }]
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ]


def generate_launch_description():
    """Generate launch description with URDF file reading"""
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

    return LaunchDescription([
        urdf_file_arg,
        use_gui_arg,
        rviz_config_arg,
        OpaqueFunction(function=launch_setup)
    ])
