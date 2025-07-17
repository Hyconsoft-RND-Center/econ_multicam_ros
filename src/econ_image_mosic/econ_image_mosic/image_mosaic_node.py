#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock
import time
import gc
import subprocess

try:
    import vpi
    VPI_AVAILABLE = True
except ImportError:
    VPI_AVAILABLE = False

class ImageMosaicNode(Node):
    def __init__(self):
        super().__init__('image_mosaic_node')
        
        # CvBridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Lock for thread safety
        self.lock = Lock()
        
        # Storage for latest images from each camera
        self.images = {}
        self.image_timestamps = {}
        
        # Parameters
        self.declare_parameter('mosaic_width', 1280)
        self.declare_parameter('mosaic_height', 720)
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('use_hardware_accel', True)
        self.declare_parameter('hardware_backend', 'auto')  # auto, vpi, gstreamer, cuda, cpu
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('use_compressed', True)
        
        self.mosaic_width = self.get_parameter('mosaic_width').get_parameter_value().integer_value
        self.mosaic_height = self.get_parameter('mosaic_height').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_hardware_accel = self.get_parameter('use_hardware_accel').get_parameter_value().bool_value
        self.hardware_backend = self.get_parameter('hardware_backend').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        
        # Camera topics
        self.camera_topics = [
            '/dev/video0/image_raw',
            '/dev/video1/image_raw',
            '/dev/video2/image_raw',
            '/dev/video3/image_raw'
        ]
        
        # QoS 설정 - BEST_EFFORT로 호환성 문제 해결
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to camera image topics
        self.subscribers = []
        for i, topic in enumerate(self.camera_topics):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cam_id=i: self.image_callback(msg, cam_id),
                qos_profile
            )
            self.subscribers.append(sub)
        
        # Publisher for mosaic image
        if self.use_compressed:
            self.mosaic_publisher = self.create_publisher(
                CompressedImage, 
                '/camera/mosaic/image_raw/compressed', 
                qos_profile
            )
        else:
            self.mosaic_publisher = self.create_publisher(
                Image, 
                '/camera/mosaic/image_raw', 
                qos_profile
            )
        
        # Timer for publishing mosaic
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.publish_mosaic
        )
        
        # Calculate individual image dimensions for 2x2 grid
        self.cell_width = self.mosaic_width // 2
        self.cell_height = self.mosaic_height // 2
        
        # 하드웨어 가속 초기화
        self.hw_accel_enabled = False
        self.accel_type = "CPU"
        self.vpi_stream = None
        self.vpi_backend = None
        
        if self.use_hardware_accel:
            self.init_hardware_acceleration()
        
        # 성능 정보 출력
        self.get_logger().info(f'=== 이미지 모자이크 노드 시작 ===')
        self.get_logger().info(f'출력 해상도: {self.mosaic_width}x{self.mosaic_height}')
        self.get_logger().info(f'게시 주파수: {self.publish_rate}Hz')
        self.get_logger().info(f'하드웨어 가속: {self.accel_type}')
        self.get_logger().info(f'압축 모드: {"활성화" if self.use_compressed else "비활성화"}')
        self.get_logger().info(f'JPEG 품질: {self.jpeg_quality}%')

    def init_hardware_acceleration(self):
        """하드웨어 가속 초기화"""
        try:
            if self.hardware_backend == 'auto' or self.hardware_backend == 'vpi':
                if VPI_AVAILABLE:
                    # VPI 백엔드 우선순위: CUDA > CPU (PVA, VIC는 현재 제약사항으로 제외)
                    backends_to_try = [
                        (vpi.Backend.CUDA, "VPI-CUDA"),
                        (vpi.Backend.CPU, "VPI-CPU")
                    ]
                    
                    for backend, name in backends_to_try:
                        try:
                            # VPI 스트림 생성 테스트
                            test_stream = vpi.Stream(backend)
                            
                            # 간단한 작업으로 백엔드 테스트 (올바른 문법)
                            test_img = vpi.asimage(np.zeros((100, 100, 3), dtype=np.uint8))
                            test_resized = test_img.rescale((50, 50), backend=backend, interp=vpi.Interp.LINEAR, stream=test_stream)
                            test_stream.sync()
                            
                            # 성공하면 이 백엔드 사용
                            self.vpi_stream = vpi.Stream(backend)
                            self.vpi_backend = backend
                            self.hw_accel_enabled = True
                            self.accel_type = name
                            self.get_logger().info(f'{name} 백엔드 활성화 성공')
                            break
                            
                        except Exception as e:
                            self.get_logger().debug(f'{name} 백엔드 테스트 실패: {str(e)}')
                            continue
                
            # VPI 실패시 OpenCV CUDA 시도
            if not self.hw_accel_enabled and (self.hardware_backend == 'auto' or self.hardware_backend == 'cuda'):
                try:
                    if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                        # CUDA 테스트
                        test_mat = cv2.cuda_GpuMat()
                        test_img = np.zeros((100, 100, 3), dtype=np.uint8)
                        test_mat.upload(test_img)
                        test_resized = cv2.cuda.resize(test_mat, (50, 50))
                        test_result = test_resized.download()
                        
                        self.hw_accel_enabled = True
                        self.accel_type = "OpenCV-CUDA"
                        self.get_logger().info('OpenCV CUDA 가속 활성화 성공')
                except Exception as e:
                    self.get_logger().debug(f'OpenCV CUDA 가속 테스트 실패: {str(e)}')
            
            # 모든 하드웨어 가속 실패시 CPU 사용
            if not self.hw_accel_enabled:
                self.accel_type = "CPU"
                self.get_logger().info('CPU 모드로 실행')
                
        except Exception as e:
            self.get_logger().warn(f'하드웨어 가속 초기화 실패: {str(e)}')
            self.hw_accel_enabled = False
            self.accel_type = "CPU"

    def image_callback(self, msg, camera_id):
        """카메라 이미지 콜백 함수"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.lock:
                # 기존 이미지 메모리 해제
                if camera_id in self.images:
                    del self.images[camera_id]
                
                self.images[camera_id] = cv_image
                self.image_timestamps[camera_id] = time.time()
                
        except Exception as e:
            self.get_logger().error(f'카메라 {camera_id} 이미지 처리 오류: {str(e)}')

    def resize_image_vpi(self, image, target_width, target_height):
        """VPI를 사용한 이미지 리사이징"""
        try:
            # NumPy array를 VPI 이미지로 변환
            vpi_img = vpi.asimage(image)
            
            # VPI로 리사이즈 (올바른 문법)
            resized_vpi = vpi_img.rescale(
                (target_width, target_height), 
                backend=self.vpi_backend,
                interp=vpi.Interp.LINEAR, 
                stream=self.vpi_stream
            )
            
            # 동기화 후 NumPy 배열로 변환
            self.vpi_stream.sync()
            resized_np = resized_vpi.cpu()
            
            return resized_np
            
        except Exception as e:
            self.get_logger().warn(f'VPI 리사이징 실패, CPU로 대체: {str(e)}')
            return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_LINEAR)

    def resize_image_cuda(self, image, target_width, target_height):
        """CUDA를 사용한 이미지 리사이징"""
        try:
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(image)
            gpu_resized = cv2.cuda.resize(gpu_img, (target_width, target_height))
            result = gpu_resized.download()
            
            # GPU 메모리 정리
            del gpu_img, gpu_resized
            
            return result
        except Exception as e:
            self.get_logger().warn(f'CUDA 리사이징 실패, CPU로 대체: {str(e)}')
            return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_LINEAR)

    def resize_image_cpu(self, image, target_width, target_height):
        """CPU를 사용한 이미지 리사이징"""
        return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_LINEAR)

    def create_mosaic(self, images_dict):
        """4개의 이미지를 2x2 모자이크로 결합"""
        try:
            # Create empty mosaic canvas
            mosaic = np.zeros((self.mosaic_height, self.mosaic_width, 3), dtype=np.uint8)
            
            # Define positions for 2x2 grid
            positions = [
                (0, 0),                                    # Top-left
                (self.cell_width, 0),                      # Top-right  
                (0, self.cell_height),                     # Bottom-left
                (self.cell_width, self.cell_height)        # Bottom-right
            ]
            
            for cam_id in range(4):
                if cam_id in images_dict:
                    image = images_dict[cam_id]
                    h, w = image.shape[:2]
                    
                    # 크기 조정이 필요한지 확인
                    if w != self.cell_width or h != self.cell_height:
                        # 하드웨어 가속 백엔드 선택
                        if self.hw_accel_enabled:
                            if self.vpi_stream is not None:
                                resized = self.resize_image_vpi(image, self.cell_width, self.cell_height)
                            elif self.accel_type == "OpenCV-CUDA":
                                resized = self.resize_image_cuda(image, self.cell_width, self.cell_height)
                            else:
                                resized = self.resize_image_cpu(image, self.cell_width, self.cell_height)
                        else:
                            resized = self.resize_image_cpu(image, self.cell_width, self.cell_height)
                    else:
                        resized = image
                    
                    # Place resized image in mosaic
                    x, y = positions[cam_id]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = resized
                    
                    # 메모리 정리
                    if resized is not image:
                        del resized
                        
                else:
                    # Fill with black if camera not available
                    x, y = positions[cam_id]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = 0
                    
            return mosaic
            
        except Exception as e:
            self.get_logger().error(f'모자이크 생성 오류: {str(e)}')
            return None

    def compress_image_gstreamer(self, image):
        """GStreamer NVIDIA JPEG 인코더 사용 (실험적)"""
        try:
            # 임시로 OpenCV JPEG 사용
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_img = cv2.imencode('.jpg', image, encode_param)
            return success, encoded_img
        except Exception as e:
            self.get_logger().warn(f'GStreamer JPEG 인코딩 실패: {str(e)}')
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            return cv2.imencode('.jpg', image, encode_param)

    def publish_mosaic(self):
        """모자이크 이미지를 게시"""
        try:
            with self.lock:
                # Check if we have recent images from cameras
                current_time = time.time()
                valid_images = {}
                
                for cam_id in range(4):
                    if (cam_id in self.images and 
                        cam_id in self.image_timestamps and
                        current_time - self.image_timestamps[cam_id] < 2.0):
                        valid_images[cam_id] = self.images[cam_id]
                
                if len(valid_images) == 0:
                    return
                    
            # Create mosaic
            mosaic = self.create_mosaic(valid_images)
            if mosaic is None:
                return
                
            # Publish with compression or standard format
            try:
                if self.use_compressed:
                    # 압축 이미지로 게시
                    success, encoded_img = self.compress_image_gstreamer(mosaic)
                    
                    if success:
                        compressed_msg = CompressedImage()
                        compressed_msg.header.stamp = self.get_clock().now().to_msg()
                        compressed_msg.header.frame_id = 'mosaic_camera'
                        compressed_msg.format = 'jpeg'
                        compressed_msg.data = encoded_img.tobytes()
                        
                        self.mosaic_publisher.publish(compressed_msg)
                        
                        # 메모리 정리
                        del encoded_img
                    else:
                        self.get_logger().warn('JPEG 인코딩 실패')
                else:
                    # 일반 이미지로 게시
                    mosaic_msg = self.bridge.cv2_to_imgmsg(mosaic, 'bgr8')
                    mosaic_msg.header.stamp = self.get_clock().now().to_msg()
                    mosaic_msg.header.frame_id = 'mosaic_camera'
                    
                    self.mosaic_publisher.publish(mosaic_msg)
                
                # 메모리 정리
                del mosaic
                
                # 주기적으로 가비지 컬렉션 실행
                if hasattr(self, '_gc_counter'):
                    self._gc_counter += 1
                else:
                    self._gc_counter = 0
                
                if self._gc_counter % 50 == 0:  # 50번마다 GC 실행
                    gc.collect()
                
            except Exception as e:
                self.get_logger().error(f'이미지 게시 오류: {str(e)}')
                
        except Exception as e:
            self.get_logger().error(f'모자이크 게시 오류: {str(e)}')

    def __del__(self):
        """소멸자 - 리소스 정리"""
        if hasattr(self, 'vpi_stream') and self.vpi_stream:
            del self.vpi_stream

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageMosaicNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'노드 실행 오류: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 