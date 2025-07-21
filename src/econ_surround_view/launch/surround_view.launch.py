#!/usr/bin/env python3

"""
Jetson AGX Orin 서라운드 뷰 시스템 런치 파일
4개 카메라 (전후좌우)에서 영상을 받아 360도 서라운드 뷰를 생성합니다.

사용법:
ros2 launch econ_surround_view surround_view.launch.py

옵션:
- use_sim:=true/false (시뮬레이션 모드)
- enable_vpi:=true/false (VPI 가속 활성화)
- output_width:=1024 (출력 이미지 너비)
- output_height:=1024 (출력 이미지 높이)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로
    pkg_econ_surround_view = get_package_share_directory('econ_surround_view')
    
    # Launch arguments
    declare_use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='시뮬레이션 모드 사용 여부'
    )
    
    declare_enable_vpi_arg = DeclareLaunchArgument(
        'enable_vpi',
        default_value='true',
        description='VPI 가속 활성화 여부'
    )
    
    declare_enable_distortion_correction_arg = DeclareLaunchArgument(
        'enable_distortion_correction',
        default_value='true',
        description='렌즈 왜곡 보정 활성화 여부'
    )
    
    declare_output_width_arg = DeclareLaunchArgument(
        'output_width',
        default_value='1024',
        description='서라운드 뷰 출력 이미지 너비'
    )
    
    declare_output_height_arg = DeclareLaunchArgument(
        'output_height',
        default_value='1024',
        description='서라운드 뷰 출력 이미지 높이'
    )
    
    declare_calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('econ_surround_view'),
            'config',
            'camera_calibration.yaml'
        ]),
        description='카메라 캘리브레이션 파일 경로'
    )
    
    declare_config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('econ_surround_view'),
            'config',
            'surround_view_params.yaml'
        ]),
        description='서라운드 뷰 설정 파일 경로'
    )

    # 카메라 토픽 설정
    declare_front_topic_arg = DeclareLaunchArgument(
        'front_camera_topic',
        default_value='/dev/video0/image_raw',
        description='전방 카메라 토픽'
    )
    
    declare_back_topic_arg = DeclareLaunchArgument(
        'back_camera_topic',
        default_value='/dev/video1/image_raw',
        description='후방 카메라 토픽'
    )
    
    declare_left_topic_arg = DeclareLaunchArgument(
        'left_camera_topic',
        default_value='/dev/video2/image_raw',
        description='좌측 카메라 토픽'
    )
    
    declare_right_topic_arg = DeclareLaunchArgument(
        'right_camera_topic',
        default_value='/dev/video3/image_raw',
        description='우측 카메라 토픽'
    )

    # 서라운드 뷰 노드
    surround_view_node = Node(
        package='econ_surround_view',
        executable='surround_view_node',
        name='surround_view_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # 카메라 토픽 설정
                'front_camera_topic': LaunchConfiguration('front_camera_topic'),
                'back_camera_topic': LaunchConfiguration('back_camera_topic'),
                'left_camera_topic': LaunchConfiguration('left_camera_topic'),
                'right_camera_topic': LaunchConfiguration('right_camera_topic'),
                
                # 출력 설정
                'output_topic': '/surround_view/image_raw',
                'output_width': LaunchConfiguration('output_width'),
                'output_height': LaunchConfiguration('output_height'),
                
                # 처리 옵션
                'enable_vpi_processing': LaunchConfiguration('enable_vpi'),
                'enable_lens_distortion_correction': LaunchConfiguration('enable_distortion_correction'),
                
                # 캘리브레이션
                'calibration_file': LaunchConfiguration('calibration_file'),
                
                # 성능 설정
                'max_queue_size': 10,
                'processing_threads': 4,
                
                # 블렌딩 설정
                'blend_alpha': 0.5,
                'enable_grid_overlay': True,
                'grid_spacing': 50,
                
                # 로깅 레벨
                'use_sim_time': LaunchConfiguration('use_sim'),
            }
        ],
        remappings=[
            # 필요 시 토픽 리매핑 추가
        ]
    )

    # 시뮬레이션 카메라 노드들 (옵션)
    sim_camera_nodes = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_sim')),
        actions=[
            Node(
                package='image_publisher',
                executable='image_publisher',
                name='front_camera_sim',
                parameters=[{
                    'frame_id': 'front_camera',
                    'publish_rate': 30.0,
                }],
                remappings=[('image', '/dev/video0/image_raw')],
                arguments=[PathJoinSubstitution([
                    FindPackageShare('econ_surround_view'),
                    'test_images',
                    'front_camera.jpg'
                ])]
            ),
            Node(
                package='image_publisher',
                executable='image_publisher',
                name='back_camera_sim',
                parameters=[{
                    'frame_id': 'back_camera',
                    'publish_rate': 30.0,
                }],
                remappings=[('image', '/dev/video1/image_raw')],
                arguments=[PathJoinSubstitution([
                    FindPackageShare('econ_surround_view'),
                    'test_images',
                    'back_camera.jpg'
                ])]
            ),
            Node(
                package='image_publisher',
                executable='image_publisher',
                name='left_camera_sim',
                parameters=[{
                    'frame_id': 'left_camera',
                    'publish_rate': 30.0,
                }],
                remappings=[('image', '/dev/video2/image_raw')],
                arguments=[PathJoinSubstitution([
                    FindPackageShare('econ_surround_view'),
                    'test_images',
                    'left_camera.jpg'
                ])]
            ),
            Node(
                package='image_publisher',
                executable='image_publisher',
                name='right_camera_sim',
                parameters=[{
                    'frame_id': 'right_camera',
                    'publish_rate': 30.0,
                }],
                remappings=[('image', '/dev/video3/image_raw')],
                arguments=[PathJoinSubstitution([
                    FindPackageShare('econ_surround_view'),
                    'test_images',
                    'right_camera.jpg'
                ])]
            ),
        ]
    )

    # RViz2 시각화 (옵션)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('econ_surround_view'),
        'config',
        'surround_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='surround_view_rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('use_sim'), '" == "true"'])
        )
    )

    # 이미지 뷰어 (디버깅용)
    image_viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='surround_view_viewer',
        arguments=['/surround_view/image_raw'],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('use_sim'), '" == "true"'])
        )
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_arg,
        declare_enable_vpi_arg,
        declare_enable_distortion_correction_arg,
        declare_output_width_arg,
        declare_output_height_arg,
        declare_calibration_file_arg,
        declare_config_file_arg,
        declare_front_topic_arg,
        declare_back_topic_arg,
        declare_left_topic_arg,
        declare_right_topic_arg,
        
        # Nodes
        surround_view_node,
        sim_camera_nodes,
        rviz_node,
        # image_viewer_node,  # 필요시 주석 해제
    ]) 