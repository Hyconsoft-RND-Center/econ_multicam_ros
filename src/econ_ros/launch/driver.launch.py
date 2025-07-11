#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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
    
    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1920',
        description='Camera frame width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1080',
        description='Camera frame height'
    )
    
    no_display_arg = DeclareLaunchArgument(
        'no_display',
        default_value='1',
        description='Disable display (1=disable, 0=enable)'
    )
    
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='0',
        description='Record video (1=enable, 0=disable)'
    )
    
    sync_arg = DeclareLaunchArgument(
        'sync',
        default_value='1',
        description='Frame sync mode (0=enable, 1=disable)'
    )

    encoding_arg = DeclareLaunchArgument(
        'encoding',
        default_value='jpeg',
        description='Image encoding type: jpeg|rgb|i420|uyvy'
    )
    
    # ExecuteProcess with arguments
    econ_ros_node = ExecuteProcess(
        cmd=[
            executable_path,
            '-w', LaunchConfiguration('width'),
            '-h', LaunchConfiguration('height'),
            '-d', LaunchConfiguration('no_display'),
            '-r', LaunchConfiguration('record'),
            '-s', LaunchConfiguration('sync'),
            '-e', LaunchConfiguration('encoding')
        ],
        output='screen'
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        no_display_arg,
        record_arg,
        sync_arg,
        encoding_arg,
        econ_ros_node
    ]) 
