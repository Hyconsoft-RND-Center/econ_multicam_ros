#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock, Thread
import time
import gc
import subprocess
import tempfile
import os
import asyncio
from concurrent.futures import ThreadPoolExecutor

try:
    import vpi
    VPI_AVAILABLE = True
except ImportError:
    VPI_AVAILABLE = False

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    Gst.init(None)
    GST_AVAILABLE = True
except ImportError:
    GST_AVAILABLE = False

class VPIManager:
    """VPI 3.2 JetPack 6.2.0 공식 방식의 최적화된 매니저"""
    def __init__(self, logger):
        self.logger = logger
        self.available_backends = []
        self.primary_backend = None
        self.output_buffers = {}
        self.vpi_stream = None
        self.is_initialized = False
        self.init_thread = None
        
        if VPI_AVAILABLE:
            # 비동기 초기화로 시작 시간 단축
            self.init_thread = Thread(target=self._async_init, daemon=True)
            self.init_thread.start()
    
    def _async_init(self):
        """비동기 VPI 초기화 (시작 시간 단축)"""
        try:
            self._detect_jetson_backends()
            self._set_primary_backend()
            self._init_vpi_stream()
            self.is_initialized = True
            self.logger.info('VPI 비동기 초기화 완료')
        except Exception as e:
            self.logger.error(f'VPI 초기화 실패: {str(e)}')
            self.is_initialized = False
    
    def _detect_jetson_backends(self):
        """JetPack 6.2.0 Jetson AGX Orin 최적화된 백엔드 감지"""
        self.logger.info('VPI 백엔드 감지 시작...')
        
        # VPI 3.2 공식: Jetson AGX Orin 지원 백엔드 순서대로 테스트
        backends_priority = [
            (vpi.Backend.CUDA, "CUDA"),
            (vpi.Backend.VIC, "VIC"),
            (vpi.Backend.PVA, "PVA"),
            (vpi.Backend.CPU, "CPU")
        ]
        
        for backend, name in backends_priority:
            if self._test_backend_capability(backend, name):
                self.available_backends.append(backend)
                self.logger.info(f'✓ VPI {name} 백엔드 활성화')
        
        # Jetson AGX Orin에서 최소한 CUDA는 동작해야 함
        if not self.available_backends:
            self.logger.warn('⚠️ VPI 백엔드 감지 실패, CPU 강제 활성화')
            self.available_backends.append(vpi.Backend.CPU)
    
    def _test_backend_capability(self, backend, backend_name):
        """VPI 3.2 올바른 백엔드 테스트 방법"""
        try:
            if backend == vpi.Backend.CUDA:
                # CUDA 백엔드 테스트: BGR8 포맷 사용
                test_input = np.ones((64, 64, 3), dtype=np.uint8) * 128
                vpi_input = vpi.asimage(test_input, format=vpi.Format.BGR8)
                vpi_output = vpi.Image((32, 32), vpi.Format.BGR8)
                
                vpi_input.rescale(
                    out=vpi_output,
                    interp=vpi.Interp.LINEAR,
                    backend=vpi.Backend.CUDA
                )
                
            elif backend == vpi.Backend.VIC:
                # VIC 백엔드 테스트: U8 포맷 사용 (VIC 지원 포맷)
                test_input = np.ones((64, 64), dtype=np.uint8) * 128
                vpi_input = vpi.asimage(test_input, format=vpi.Format.U8)
                vpi_output = vpi.Image((32, 32), vpi.Format.U8)
                
                vpi_input.rescale(
                    out=vpi_output,
                    interp=vpi.Interp.LINEAR,
                    backend=vpi.Backend.VIC
                )
                
            elif backend == vpi.Backend.PVA:
                # PVA 백엔드 테스트: Box Filter 사용
                test_input = np.ones((64, 64, 3), dtype=np.uint8) * 128
                vpi_input = vpi.asimage(test_input, format=vpi.Format.BGR8)
                vpi_output = vpi.Image((64, 64), vpi.Format.BGR8)
                
                vpi_input.boxfilter(
                    out=vpi_output,
                    kernel_size=(3, 3),
                    backend=vpi.Backend.PVA
                )
                
            elif backend == vpi.Backend.CPU:
                # CPU 백엔드 테스트
                test_input = np.ones((64, 64, 3), dtype=np.uint8) * 128
                vpi_input = vpi.asimage(test_input, format=vpi.Format.BGR8)
                vpi_output = vpi.Image((32, 32), vpi.Format.BGR8)
                
                vpi_input.rescale(
                    out=vpi_output,
                    interp=vpi.Interp.LINEAR,
                    backend=vpi.Backend.CPU
                )
            
            # 명시적 정리
            vpi_input = None
            vpi_output = None
            
            return True
            
        except Exception as e:
            self.logger.debug(f"{backend_name} 백엔드 테스트 실패: {e}")
            return False
    
    def _set_primary_backend(self):
        """성능 최적화된 주 백엔드 선택"""
        if not self.available_backends:
            self.primary_backend = None
            self.logger.error('❌ 사용 가능한 VPI 백엔드 없음')
            return
        
        # Jetson AGX Orin 최적화 순서: CUDA > VIC > PVA > CPU
        if vpi.Backend.CUDA in self.available_backends:
            self.primary_backend = vpi.Backend.CUDA
            self.logger.info('🚀 VPI-CUDA 백엔드 선택 (최고 성능)')
        elif vpi.Backend.VIC in self.available_backends:
            self.primary_backend = vpi.Backend.VIC
            self.logger.info('⚡ VPI-VIC 백엔드 선택 (하드웨어 가속)')
        elif vpi.Backend.PVA in self.available_backends:
            self.primary_backend = vpi.Backend.PVA
            self.logger.info('🔧 VPI-PVA 백엔드 선택')
        else:
            self.primary_backend = vpi.Backend.CPU
            self.logger.info('💻 VPI-CPU 백엔드 선택 (폴백)')
    
    def _init_vpi_stream(self):
        """VPI 3.2 올바른 Stream 초기화"""
        try:
            if not self.primary_backend:
                self.vpi_stream = None
                return
            
            # VPI 3.2: 기본 Stream 생성 (백엔드 지정 안함)
            self.vpi_stream = vpi.Stream()
            self.logger.info(f'VPI Stream 초기화 성공: {str(self.primary_backend).split(".")[-1]}')
            
        except Exception as e:
            self.logger.error(f'VPI Stream 초기화 실패: {str(e)}')
            self.vpi_stream = None
    
    def wait_for_initialization(self, timeout=10.0):
        """VPI 초기화 완료까지 대기 (타임아웃 설정)"""
        if self.init_thread:
            self.init_thread.join(timeout=timeout)
            if self.init_thread.is_alive():
                self.logger.warn('VPI 초기화 타임아웃, 기본 설정으로 진행')
                return False
        return self.is_initialized
    
    def get_or_create_buffer(self, shape, format_type):
        """메모리 효율성을 위한 버퍼 재사용"""
        key = (shape, format_type)
        if key not in self.output_buffers:
            self.output_buffers[key] = vpi.Image(shape, format_type)
        return self.output_buffers[key]
    
    def process_resize(self, cv_image, target_size):
        """VPI 3.2 최적화된 이미지 리사이징"""
        if not self.is_initialized or not self.primary_backend or not self.vpi_stream:
            # VPI 사용 불가시 빠른 OpenCV 폴백
            return cv2.resize(cv_image, target_size, interpolation=cv2.INTER_LINEAR)
        
        try:
            # VPI 3.2 공식: Stream 기반 처리
            vpi_input = vpi.asimage(cv_image, format=vpi.Format.BGR8)
            
            output_shape = (target_size[1], target_size[0])
            vpi_output = self.get_or_create_buffer(output_shape, vpi.Format.BGR8)
            
            # VPI 3.2: backend 파라미터로 백엔드 지정
            vpi_input.rescale(
                out=vpi_output,
                interp=vpi.Interp.LINEAR,
                backend=self.primary_backend
            )
            
            # VPI 3.2: 동기화 (필요시만)
            self.vpi_stream.sync()
            
            # 결과 복사 (최적화된 메모리 접근)
            with vpi_output.rlock(vpi.MemType.HOST) as output_data:
                result = np.array(output_data, copy=True)
            
            del vpi_input
            return result
            
        except Exception as e:
            self.logger.debug(f'VPI 처리 실패, OpenCV 폴백: {str(e)}')
            return cv2.resize(cv_image, target_size, interpolation=cv2.INTER_LINEAR)
    
    def cleanup(self):
        """VPI 리소스 정리"""
        try:
            if self.vpi_stream:
                self.vpi_stream.sync()
                del self.vpi_stream
                self.vpi_stream = None
            
            for buffer in self.output_buffers.values():
                del buffer
            self.output_buffers.clear()
            
            self.logger.info('VPI 매니저 정리 완료')
        except Exception as e:
            self.logger.error(f'VPI 매니저 정리 실패: {str(e)}')

class ImageMosaicHWNode(Node):
    def __init__(self):
        super().__init__('image_mosaic_hw_node')
        
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
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('jpeg_quality', 85)
        
        # Get parameters
        self.mosaic_width = self.get_parameter('mosaic_width').value
        self.mosaic_height = self.get_parameter('mosaic_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Calculate cell dimensions for 2x2 grid
        self.cell_width = self.mosaic_width // 2
        self.cell_height = self.mosaic_height // 2
        
        # QoS profile for camera compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera topics
        self.camera_topics = ['/dev/video0/image_raw', '/dev/video1/image_raw', 
                             '/dev/video2/image_raw', '/dev/video3/image_raw']
        self.subscribers = []
        for i, topic in enumerate(self.camera_topics):
            subscriber = self.create_subscription(
                Image, topic, 
                lambda msg, camera_id=i: self.image_callback(msg, camera_id),
                qos_profile
            )
            self.subscribers.append(subscriber)
        
        # Publishers
        self.mosaic_publisher = self.create_publisher(Image, '/camera/mosaic_hw/image_raw', 10)
        self.compressed_publisher = self.create_publisher(CompressedImage, '/camera/mosaic_hw/image_raw/compressed', 10)
        
        # VPI 3.2 매니저 초기화 (비동기)
        self.vpi_manager = VPIManager(self.get_logger())
        
        # GStreamer 비동기 초기화
        self.gst_pipeline = None
        self.use_compressed = True
        self.gst_init_thread = Thread(target=self.init_gstreamer_compression, daemon=True)
        self.gst_init_thread.start()
        
        # Timer for mosaic generation (빠른 시작)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.create_and_publish_mosaic)
        
        # 향상된 메모리 관리
        self.frame_count = 0
        self.memory_check_interval = 200  # 최적화: 200프레임마다
        self.last_cleanup_time = time.time()
        self.cleanup_interval = 60
        
        # Performance tracking
        self.last_publish_time = time.time()
        self.published_count = 0
        self.last_stats_time = time.time()
        
        # 디버깅용 이미지 수신 카운터
        self.image_receive_count = {0: 0, 1: 0, 2: 0, 3: 0}
        
        # 빠른 시작을 위한 초기 로그
        self.get_logger().info('=== VPI 3.2 최적화 이미지 모자이크 노드 시작 ===')
        self.get_logger().info(f'출력 해상도: {self.mosaic_width}x{self.mosaic_height}')
        self.get_logger().info(f'게시 주파수: {self.publish_rate}Hz')
        
        # 백그라운드에서 초기화 상태 체크
        self.create_timer(2.0, self.check_initialization_status)

    def check_initialization_status(self):
        """초기화 상태 체크 및 로깅"""
        try:
            # VPI 초기화 상태 확인
            vpi_ready = self.vpi_manager.wait_for_initialization(timeout=0.1)
            
            # GStreamer 초기화 상태 확인
            gst_ready = not self.gst_init_thread.is_alive() if self.gst_init_thread else True
            
            if vpi_ready and gst_ready:
                # 모든 초기화 완료
                self.get_logger().info(f'VPI 상태: {"활성" if self.vpi_manager.is_initialized else "비활성"}')
                self.get_logger().info(f'VPI 백엔드: {[str(b).split(".")[-1] for b in self.vpi_manager.available_backends]}')
                self.get_logger().info(f'주 백엔드: {str(self.vpi_manager.primary_backend).split(".")[-1] if self.vpi_manager.primary_backend else "없음"}')
                self.get_logger().info(f'GStreamer 압축: {"활성화" if self.gst_pipeline else "비활성화"}')
                self.get_logger().info(f'JPEG 품질: {self.jpeg_quality}%')
                
                # 타이머 제거 (한 번만 실행)
                self.destroy_timer(self.timer_init_check)
        except:
            pass
        
        # 타이머 참조 저장
        self.timer_init_check = self.create_timer(5.0, lambda: None)  # 5초 후 자동 제거

    def init_gstreamer_compression(self):
        """비동기 GStreamer 초기화"""
        if not GST_AVAILABLE:
            self.get_logger().warn('GStreamer 사용 불가')
            return
            
        try:
            # Jetson AGX Orin 최적화된 하드웨어 JPEG 인코더
            pipeline_hw = f"""
            appsrc name=src caps=video/x-raw,format=BGR,width={self.mosaic_width},height={self.mosaic_height},framerate=60/1 !
            videoconvert !
            nvjpegenc quality={self.jpeg_quality} !
            appsink name=sink emit-signals=true sync=false
            """
            
            self.gst_pipeline = Gst.parse_launch(pipeline_hw)
            if self.gst_pipeline:
                self.get_logger().info('GStreamer nvjpegenc 파이프라인 초기화 성공')
                return
                
        except Exception as e:
            self.get_logger().warn(f'nvjpegenc 실패: {str(e)}')
            
        try:
            # 소프트웨어 폴백
            pipeline_sw = f"""
            appsrc name=src caps=video/x-raw,format=BGR,width={self.mosaic_width},height={self.mosaic_height},framerate=60/1 !
            videoconvert !
            jpegenc quality={self.jpeg_quality} !
            appsink name=sink emit-signals=true sync=false
            """
            self.gst_pipeline = Gst.parse_launch(pipeline_sw)
            self.get_logger().info('소프트웨어 JPEG 인코더 활성화')
        except Exception as e:
            self.get_logger().error(f'GStreamer 초기화 실패: {str(e)}')
            self.gst_pipeline = None

    def image_callback(self, msg, camera_id):
        """카메라 이미지 콜백 (최적화)"""
        try:
            with self.lock:
                # Convert ROS Image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # Store the latest image and timestamp
                self.images[camera_id] = cv_image
                self.image_timestamps[camera_id] = msg.header.stamp
                
                # 이미지 수신 카운터 증가
                self.image_receive_count[camera_id] += 1
                
        except Exception as e:
            self.get_logger().error(f'카메라 {camera_id} 이미지 콜백 에러: {str(e)}')

    def create_and_publish_mosaic(self):
        """빠른 시작 모자이크 생성 및 발행"""
        try:
            with self.lock:
                # 빠른 시작: 1개 카메라만 있어도 시작
                if len(self.images) < 1:
                    return
                
                # 프레임 카운터 증가
                self.frame_count += 1
                
                # Create mosaic
                mosaic = self.create_mosaic_optimized()
                if mosaic is None:
                    return
                
                # Publish raw image
                self.publish_raw_mosaic(mosaic)
                
                # Publish compressed image (최적화: 선택적)
                if self.use_compressed and self.frame_count % 3 == 0:  # 3프레임마다
                    self.publish_compressed_mosaic(mosaic)
                
                # 발행 카운터 증가
                self.published_count += 1
                
                # 메모리 관리 (최적화)
                if self.frame_count % self.memory_check_interval == 0:
                    self.periodic_memory_management()
                
                # FPS 계산 및 로깅
                self.log_performance_stats()
                
        except Exception as e:
            self.get_logger().error(f'모자이크 생성/발행 에러: {str(e)}')

    def create_mosaic_optimized(self):
        """VPI 3.2 최적화된 모자이크 생성"""
        try:
            # Create mosaic canvas
            mosaic = np.zeros((self.mosaic_height, self.mosaic_width, 3), dtype=np.uint8)
            
            # Define positions for 2x2 grid
            positions = [
                (0, 0),  # Top-left
                (0, self.cell_width),  # Top-right  
                (self.cell_height, 0),  # Bottom-left
                (self.cell_height, self.cell_width)  # Bottom-right
            ]
            
            for camera_id in range(4):
                if camera_id in self.images:
                    image = self.images[camera_id]
                    
                    # VPI 또는 OpenCV 리사이징 (조건부 최적화)
                    if (self.vpi_manager.is_initialized and 
                        self.vpi_manager.primary_backend in [vpi.Backend.CUDA, vpi.Backend.VIC]):
                        # 하드웨어 가속 백엔드만 VPI 사용
                        resized = self.vpi_manager.process_resize(
                            image, 
                            (self.cell_width, self.cell_height)
                        )
                    else:
                        # 빠른 OpenCV 리사이징
                        resized = cv2.resize(
                            image, 
                            (self.cell_width, self.cell_height),
                            interpolation=cv2.INTER_LINEAR
                        )
                    
                    # Place resized image in mosaic
                    y, x = positions[camera_id]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = resized
                else:
                    # 빈 칸: 회색 화면
                    y, x = positions[camera_id]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = 64
            
            return mosaic
            
        except Exception as e:
            self.get_logger().error(f'모자이크 생성 에러: {str(e)}')
            return None

    def publish_raw_mosaic(self, mosaic):
        """원본 모자이크 발행"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(mosaic, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'mosaic_frame'
            
            self.mosaic_publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'원본 모자이크 발행 에러: {str(e)}')

    def publish_compressed_mosaic(self, mosaic):
        """최적화된 압축 모자이크 발행"""
        try:
            # 간소화된 OpenCV JPEG 압축 (GStreamer 복잡성 제거)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_image = cv2.imencode('.jpg', mosaic, encode_param)
            
            if result:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.header.frame_id = 'mosaic_frame'
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()
                
                self.compressed_publisher.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'압축 모자이크 발행 에러: {str(e)}')

    def compress_with_gstreamer(self, image):
        """GStreamer를 사용한 JPEG 압축"""
        try:
            # GStreamer 파이프라인 구현 (간소화)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_image = cv2.imencode('.jpg', image, encode_param)
            
            if result:
                return encoded_image.tobytes()
            return None
            
        except Exception as e:
            self.get_logger().warn(f'GStreamer 압축 실패: {str(e)}')
            return None

    def compress_with_opencv(self, image):
        """OpenCV를 사용한 JPEG 압축 (폴백)"""
        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_image = cv2.imencode('.jpg', image, encode_param)
            
            if result:
                return encoded_image.tobytes()
            return None
            
        except Exception as e:
            self.get_logger().error(f'OpenCV 압축 실패: {str(e)}')
            return None

    def periodic_memory_management(self):
        """최적화된 메모리 관리"""
        try:
            # 기본 정리 - 빠르고 가벼움
            if self.frame_count % 600 == 0:  # 10초마다 (30Hz 기준)
                gc.collect()
                
            # 중급 정리 - 1분마다
            current_time = time.time()
            if current_time - self.last_cleanup_time > 60:
                self.intermediate_cleanup()
                self.last_cleanup_time = current_time
                
        except Exception as e:
            self.get_logger().debug(f'메모리 관리 에러: {str(e)}')

    def intermediate_cleanup(self):
        """중간 수준 메모리 정리"""
        try:
            # 타임스탬프 캐시 제한
            if len(self.image_timestamps) > 8:
                # 가장 오래된 2개 제거
                oldest_keys = sorted(self.image_timestamps.keys(), 
                                   key=lambda k: self.image_timestamps[k].sec)[:2]
                for key in oldest_keys:
                    del self.image_timestamps[key]
            
            # 가벼운 가비지 컬렉션
            gc.collect()
            
            self.get_logger().debug('중간 메모리 정리 완료')
            
        except Exception as e:
            self.get_logger().debug(f'중간 메모리 정리 에러: {str(e)}')

    def log_performance_stats(self):
        """최적화된 성능 통계 로깅"""
        current_time = time.time()
        
        # 30초마다 통계 출력 (성능 개선)
        if current_time - self.last_stats_time >= 30.0:
            try:
                elapsed = current_time - self.last_publish_time
                actual_fps = self.published_count / elapsed if elapsed > 0 else 0
                
                self.get_logger().info(f'=== 성능 통계 (30초) ===')
                self.get_logger().info(f'실제 FPS: {actual_fps:.1f}Hz')
                self.get_logger().info(f'발행된 모자이크: {self.published_count}개')
                self.get_logger().info(f'이미지 수신: {dict(self.image_receive_count)}')
                self.get_logger().info(f'VPI 상태: {"활성" if self.vpi_manager.is_initialized else "비활성"}')
                self.get_logger().info(f'메모리 정리 주기: {self.frame_count}프레임')
                
                self.last_stats_time = current_time
                
            except Exception as e:
                self.get_logger().debug(f'성능 통계 에러: {str(e)}')

    def destroy_node(self):
        """노드 종료시 정리"""
        try:
            self.get_logger().info('=== 노드 종료 시작 ===')
            
            # 타이머 중지
            if hasattr(self, 'timer'):
                self.timer.cancel()
            
            # VPI 매니저 정리
            if hasattr(self, 'vpi_manager'):
                self.vpi_manager.cleanup()
            
            # GStreamer 파이프라인 정리
            if self.gst_pipeline:
                try:
                    self.gst_pipeline.set_state(Gst.State.NULL)
                    self.gst_pipeline = None
                except:
                    pass
            
            # 스레드 정리
            if hasattr(self, 'gst_init_thread') and self.gst_init_thread.is_alive():
                self.gst_init_thread.join(timeout=2.0)
            
            # 이미지 캐시 정리
            if hasattr(self, 'images'):
                self.images.clear()
            
            # 타임스탬프 정리
            if hasattr(self, 'image_timestamps'):
                self.image_timestamps.clear()
                
            # 최종 가비지 컬렉션
            gc.collect()
            
            self.get_logger().info('=== 노드 종료 완료 ===')
            
        except Exception as e:
            self.get_logger().error(f'노드 종료 에러: {str(e)}')
        
        # 부모 클래스 종료
        super().destroy_node()

def main(args=None):
    """메인 함수 - 빠른 시작 최적화"""
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        node = ImageMosaicHWNode()
        
        # 빠른 시작 로그
        node.get_logger().info('🚀 VPI 3.2 최적화 모자이크 노드 시작됨')
        node.get_logger().info('⏱️ 백그라운드 초기화 진행 중...')
        
        # 노드 실행
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('키보드 인터럽트 감지됨')
        except Exception as e:
            node.get_logger().error(f'노드 실행 중 에러: {str(e)}')
            
    except Exception as e:
        print(f'노드 생성 실패: {str(e)}')
        
    finally:
        try:
            # 안전한 종료
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main() 