#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Width of the camera resolution'
    )
    
    height_arg = DeclareLaunchArgument(
        'height', 
        default_value='1080',
        description='Height of the camera resolution'
    )
    
    num_cam_arg = DeclareLaunchArgument(
        'num_cam',
        default_value='4',
        description='Number of cameras to use'
    )
    
    no_display_arg = DeclareLaunchArgument(
        'no_display',
        default_value='1',
        description='Set no-display streaming to 1'
    )

    no_sync_arg = DeclareLaunchArgument(
        'no_sync',
        default_value='1',
        description='Set no-sync to 1 for disable sync mode'
    )

    # Get package installation path
    package_name = 'econ_ros'
    executable_path = os.path.join(
        get_package_share_directory(package_name),
        '..',
        '..',
        'lib',
        package_name,
        'econ_ros'
    )
    
    # Simple ExecuteProcess - prioritize camera hardware safety
    econ_ros_node = ExecuteProcess(
        cmd=[
            executable_path,
            '--width', LaunchConfiguration('width'),
            '--height', LaunchConfiguration('height'),
            '--num_cam', LaunchConfiguration('num_cam'),
            '--no-display', LaunchConfiguration('no_display'),
            '--no-sync', LaunchConfiguration('no_sync')
        ],
        output='screen',
        name='econ_ros_node',
        # No respawn - let it exit cleanly
        respawn=False,
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        num_cam_arg,
        no_display_arg,
        no_sync_arg,
        econ_ros_node
    ]) 
