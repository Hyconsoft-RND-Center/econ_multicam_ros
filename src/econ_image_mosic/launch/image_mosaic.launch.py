#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    mosaic_width_arg = DeclareLaunchArgument(
        'mosaic_width',
        default_value='1280',
        description='모자이크 이미지의 너비'
    )
    
    mosaic_height_arg = DeclareLaunchArgument(
        'mosaic_height', 
        default_value='720',
        description='모자이크 이미지의 높이'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='15.0', 
        description='모자이크 이미지 게시 주파수 (Hz)'
    )
    
    use_hardware_accel_arg = DeclareLaunchArgument(
        'use_hardware_accel',
        default_value='true',
        description='하드웨어 가속 사용 여부'
    )
    
    hardware_backend_arg = DeclareLaunchArgument(
        'hardware_backend',
        default_value='auto',
        description='하드웨어 백엔드 선택: auto, vpi, cuda, cpu'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='70',
        description='JPEG 압축 품질 (1-100)'
    )
    
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='압축 이미지 게시 사용 여부'
    )

    # Image mosaic node
    image_mosaic_node = Node(
        package='econ_image_mosic',
        executable='image_mosaic_node',
        name='image_mosaic_node',
        output='screen',
        parameters=[{
            'mosaic_width': LaunchConfiguration('mosaic_width'),
            'mosaic_height': LaunchConfiguration('mosaic_height'), 
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_hardware_accel': LaunchConfiguration('use_hardware_accel'),
            'hardware_backend': LaunchConfiguration('hardware_backend'),
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
        # 메모리 및 우선순위 최적화
        additional_env={'RCUTILS_LOGGING_BUFFERED_STREAM': '1'}
    )

    return LaunchDescription([
        mosaic_width_arg,
        mosaic_height_arg, 
        publish_rate_arg,
        use_hardware_accel_arg,
        hardware_backend_arg,
        jpeg_quality_arg,
        use_compressed_arg,
        image_mosaic_node
    ]) 