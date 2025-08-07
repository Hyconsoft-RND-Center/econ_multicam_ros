#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    mosaic_width_arg = DeclareLaunchArgument(
        'mosaic_width',
        default_value='1980',
        description='모자이크 이미지의 너비'
    )
    
    mosaic_height_arg = DeclareLaunchArgument(
        'mosaic_height', 
        default_value='1080',
        description='모자이크 이미지의 높이'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='60.0', 
        description='모자이크 이미지 게시 주파수 (Hz) - 하드웨어 가속으로 30fps 목표'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='85',
        description='JPEG 압축 품질 (1-100) - nvjpegenc 하드웨어 인코더 사용'
    )
    
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='압축 이미지 게시 사용 여부'
    )

    # Hardware accelerated image mosaic node
    image_mosaic_hw_node = Node(
        package='econ_image_mosic',
        executable='image_mosaic_hw_node',
        name='image_mosaic_hw_node',
        output='screen',
        parameters=[{
            'mosaic_width': LaunchConfiguration('mosaic_width'),
            'mosaic_height': LaunchConfiguration('mosaic_height'), 
            'publish_rate': LaunchConfiguration('publish_rate'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'use_compressed': LaunchConfiguration('use_compressed')
        }],
        remappings=[
            # 입력 토픽 매핑 (필요시 변경 가능)
            # ('/dev/video0/image_raw', '/camera0/image_raw'),
            # ('/dev/video1/image_raw', '/camera1/image_raw'),
            # ('/dev/video2/image_raw', '/camera2/image_raw'),
            # ('/dev/video3/image_raw', '/camera3/image_raw'),
        ],
        # 하드웨어 가속 최적화 환경변수
        additional_env={
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
            'GST_PLUGIN_PATH': '/usr/lib/aarch64-linux-gnu/gstreamer-1.0'
        }
    )

    return LaunchDescription([
        mosaic_width_arg,
        mosaic_height_arg, 
        publish_rate_arg,
        jpeg_quality_arg,
        use_compressed_arg,
        image_mosaic_hw_node
    ]) 