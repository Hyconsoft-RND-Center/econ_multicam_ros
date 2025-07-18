#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('econ_mosaic_vpi')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Mosaic view node
    mosaic_view_node = Node(
        package='econ_mosaic_vpi',
        executable='mosaic_view_node',
        name='mosaic_view_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg='Starting VPI-accelerated Mosaic View System...'),
        mosaic_view_node
    ]) 