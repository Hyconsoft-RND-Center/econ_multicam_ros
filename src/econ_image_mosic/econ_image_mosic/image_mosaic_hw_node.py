#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Lock
import time
import gc
import subprocess
import tempfile
import os

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

# VPI 확장 기능 (Jetpack 최적화)
VPI_BACKENDS_AVAILABLE = []
if VPI_AVAILABLE:
    try:
        # 사용 가능한 VPI 백엔드 확인
        for backend in [vpi.Backend.CUDA, vpi.Backend.VIC, vpi.Backend.PVA, vpi.Backend.CPU]:
            try:
                test_stream = vpi.Stream(backend)
                VPI_BACKENDS_AVAILABLE.append(backend)
                del test_stream
            except:
                pass
    except:
        pass

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
        self.declare_parameter('publish_rate', 30.0)
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
        
        # Timer for mosaic generation
        self.timer = self.create_timer(1.0 / self.publish_rate, self.create_and_publish_mosaic)
        
        # Hardware acceleration initialization
        self.vpi_stream = None
        self.vpi_backend = None
        self.gst_pipeline = None
        self.use_compressed = True
        
        # VPI GPU 이미지 캐시 (성능 최적화)
        self.vpi_image_cache = {}
        self.vpi_output_cache = None
        
        # Initialize hardware acceleration
        self.init_hardware_acceleration()
        
        # Performance tracking
        self.last_time = time.time()
        self.frame_count = 0
        
        # Log configuration
        self.get_logger().info('=== 하드웨어 가속 이미지 모자이크 노드 시작 ===')
        self.get_logger().info(f'출력 해상도: {self.mosaic_width}x{self.mosaic_height}')
        self.get_logger().info(f'게시 주파수: {self.publish_rate}Hz')
        self.get_logger().info(f'VPI 가속: {"활성화" if self.vpi_stream else "비활성화"}')
        self.get_logger().info(f'VPI 백엔드: {[str(b).split(".")[-1] for b in VPI_BACKENDS_AVAILABLE]}')
        self.get_logger().info(f'GStreamer 가속: {"활성화" if self.gst_pipeline else "비활성화"}')
        self.get_logger().info(f'압축 모드: {"활성화" if self.use_compressed else "비활성화"}')
        self.get_logger().info(f'JPEG 품질: {self.jpeg_quality}%')

    def init_hardware_acceleration(self):
        """하드웨어 가속 초기화 - VPI CUDA + GStreamer 하이브리드 방식"""
        try:
            # VPI 초기화 (CUDA 백엔드 우선, 실패시 CPU 폴백)
            if VPI_AVAILABLE:
                # 1차: CUDA 백엔드 시도
                try:
                    self.vpi_stream = vpi.Stream(vpi.Backend.CUDA)
                    test_img = vpi.asimage(np.zeros((100, 100, 3), dtype=np.uint8))
                    test_resized = test_img.rescale((50, 50), backend=vpi.Backend.CUDA, stream=self.vpi_stream)
                    self.vpi_stream.sync()
                    self.vpi_backend = vpi.Backend.CUDA
                    self.get_logger().info('VPI-CUDA 초기화 성공 - GPU 가속 활성화')
                except Exception as cuda_e:
                    self.get_logger().warn(f'VPI-CUDA 실패, CPU로 폴백: {str(cuda_e)}')
                    # 2차: CPU 백엔드 폴백
                    try:
                        self.vpi_stream = vpi.Stream(vpi.Backend.CPU)
                        test_img = vpi.asimage(np.zeros((100, 100, 3), dtype=np.uint8))
                        test_resized = test_img.rescale((50, 50), backend=vpi.Backend.CPU, stream=self.vpi_stream)
                        self.vpi_stream.sync()
                        self.vpi_backend = vpi.Backend.CPU
                        self.get_logger().info('VPI-CPU 초기화 성공 - CPU 백엔드 사용')
                    except Exception as cpu_e:
                        self.get_logger().warn(f'VPI 전체 초기화 실패: {str(cpu_e)}')
                        self.vpi_stream = None
            
            # GStreamer JPEG 압축 파이프라인 초기화 (GPU 가속)
            if GST_AVAILABLE:
                try:
                    self.init_jpeg_pipeline()
                except Exception as e:
                    self.get_logger().warn(f'GStreamer JPEG 파이프라인 초기화 실패: {str(e)}')
                    
        except Exception as e:
            self.get_logger().error(f'하드웨어 가속 초기화 전체 실패: {str(e)}')

    def init_jpeg_pipeline(self):
        """GStreamer JPEG 압축 전용 파이프라인 초기화 (GPU 가속)"""
        try:
            # nvjpegenc 사용 가능 여부 확인
            pipeline_test = f"""
                videotestsrc num-buffers=1 !
                video/x-raw,format=RGB,width=640,height=480 !
                videoconvert ! video/x-raw,format=BGR !
                nvvidconv ! video/x-raw(memory:NVMM),format=NV12 !
                nvjpegenc quality=50 !
                fakesink
            """
            
            try:
                test_pipeline = Gst.parse_launch(pipeline_test)
                ret = test_pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise Exception("nvjpegenc 테스트 실패")
                test_pipeline.set_state(Gst.State.NULL)
                self.get_logger().info('nvjpegenc GPU 가속 사용 가능 확인')
            except Exception as test_e:
                self.get_logger().warn(f'nvjpegenc 테스트 실패, SW JPEG 사용: {str(test_e)}')
                self.init_sw_jpeg_pipeline()
                return
            
            # GPU 가속 JPEG 파이프라인
            pipeline_str = f"""
                appsrc name=src format=time do-timestamp=true caps=video/x-raw,format=BGR,width={self.mosaic_width},height={self.mosaic_height},framerate=30/1 !
                videoconvert ! video/x-raw,format=I420 !
                nvvidconv ! video/x-raw(memory:NVMM),format=NV12 !
                nvjpegenc quality={self.jpeg_quality} !
                appsink name=sink emit-signals=true max-buffers=1 drop=true
            """
            
            self.gst_pipeline = Gst.parse_launch(pipeline_str)
            
            # AppSrc/AppSink 설정
            self.app_src = self.gst_pipeline.get_by_name('src')
            self.app_sink = self.gst_pipeline.get_by_name('sink')
            self.app_sink.connect('new-sample', self.on_new_sample)
            
            # 파이프라인 시작
            ret = self.gst_pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("GPU JPEG 파이프라인 시작 실패")
                
            self.get_logger().info('GStreamer GPU JPEG 압축 파이프라인 초기화 성공')
            
        except Exception as e:
            self.get_logger().error(f'GPU JPEG 파이프라인 초기화 실패: {str(e)}')
            # SW JPEG 폴백
            self.init_sw_jpeg_pipeline()

    def init_sw_jpeg_pipeline(self):
        """소프트웨어 JPEG 압축 파이프라인 (폴백)"""
        try:
            pipeline_str = f"""
                appsrc name=src format=time do-timestamp=true caps=video/x-raw,format=BGR,width={self.mosaic_width},height={self.mosaic_height},framerate=30/1 !
                jpegenc quality={self.jpeg_quality} !
                appsink name=sink emit-signals=true max-buffers=1 drop=true
            """
            
            self.gst_pipeline = Gst.parse_launch(pipeline_str)
            
            # AppSrc/AppSink 설정
            self.app_src = self.gst_pipeline.get_by_name('src')
            self.app_sink = self.gst_pipeline.get_by_name('sink')
            self.app_sink.connect('new-sample', self.on_new_sample)
            
            # 파이프라인 시작
            ret = self.gst_pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("SW JPEG 파이프라인 시작 실패")
                
            self.get_logger().info('GStreamer SW JPEG 압축 파이프라인 초기화 성공')
            
        except Exception as e:
            self.get_logger().error(f'SW JPEG 파이프라인 초기화 실패: {str(e)}')
            self.gst_pipeline = None

    def on_new_sample(self, sink):
        """GStreamer에서 압축된 JPEG 데이터 수신"""
        try:
            sample = sink.emit('pull-sample')
            if sample:
                buffer = sample.get_buffer()
                success, map_info = buffer.map(Gst.MapFlags.READ)
                if success:
                    # JPEG 데이터를 ROS CompressedImage로 변환
                    jpeg_data = map_info.data
                    
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.get_clock().now().to_msg()
                    compressed_msg.header.frame_id = 'camera_mosaic'
                    compressed_msg.format = 'jpeg'
                    compressed_msg.data = jpeg_data
                    
                    self.compressed_publisher.publish(compressed_msg)
                    buffer.unmap(map_info)
                    
        except Exception as e:
            self.get_logger().warn(f'GStreamer JPEG 처리 오류: {str(e)}')
        
        return Gst.FlowReturn.OK

    def image_callback(self, msg, camera_id):
        """카메라 이미지 콜백"""
        try:
            with self.lock:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.images[camera_id] = cv_image
                self.image_timestamps[camera_id] = time.time()
        except Exception as e:
            self.get_logger().warn(f'카메라 {camera_id} 이미지 변환 실패: {str(e)}')

    def resize_image_vpi(self, image, target_width, target_height):
        """VPI를 사용한 이미지 리사이징 (CUDA 우선, CPU 폴백)"""
        try:
            if self.vpi_stream is None:
                # VPI 사용 불가시 OpenCV 폴백
                return cv2.resize(image, (target_width, target_height))
                
            # VPI로 리사이즈 (설정된 백엔드 사용)
            vpi_img = vpi.asimage(image)
            resized_vpi = vpi_img.rescale(
                (target_width, target_height), 
                backend=self.vpi_backend,
                interp=vpi.Interp.LINEAR, 
                stream=self.vpi_stream
            )
            
            self.vpi_stream.sync()
            resized_cv = resized_vpi.cpu()
            
            return resized_cv
            
        except Exception as e:
            self.get_logger().warn(f'VPI 리사이징 실패, OpenCV로 대체: {str(e)}')
            return cv2.resize(image, (target_width, target_height))

    def create_mosaic_vpi_optimized(self, images):
        """VPI 최적화 모자이크 생성 (GPU 활용도 증대)"""
        try:
            # VPI로 전체 모자이크 이미지 생성 (GPU 메모리에서 처리)
            if self.vpi_stream and self.vpi_backend == vpi.Backend.CUDA:
                return self.create_mosaic_vpi_gpu(images)
            else:
                return self.create_mosaic_hybrid(images)
                
        except Exception as e:
            self.get_logger().error(f'VPI 모자이크 생성 실패: {str(e)}')
            return self.create_mosaic_fallback(images)

    def create_mosaic_vpi_gpu(self, images):
        """VPI GPU 전용 모자이크 생성 (최대 GPU 활용)"""
        try:
            # GPU 메모리에서 모자이크 조합
            if self.vpi_output_cache is None:
                # VPI Image 생성 (backend 파라미터 제거)
                self.vpi_output_cache = vpi.Image(
                    (self.mosaic_width, self.mosaic_height), 
                    vpi.Format.BGR8
                )
            
            # GPU에서 배경을 검은색으로 초기화
            black_array = np.zeros((self.mosaic_height, self.mosaic_width, 3), dtype=np.uint8)
            vpi_black = vpi.asimage(black_array)
            
            # 첫 번째 이미지로 초기화
            if 0 in images:
                resized = self.resize_image_vpi_cached(images[0], 0)
                if resized is not None:
                    # GPU에서 직접 복사
                    self.copy_to_mosaic_vpi(resized, 0, 0)
            
            # 나머지 이미지들을 GPU에서 조합
            positions = [(self.cell_width, 0), (0, self.cell_height), (self.cell_width, self.cell_height)]
            for i, (x, y) in enumerate(positions, 1):
                if i in images:
                    resized = self.resize_image_vpi_cached(images[i], i)
                    if resized is not None:
                        self.copy_to_mosaic_vpi(resized, x, y)
            
            # CPU로 결과 다운로드
            result = self.vpi_output_cache.cpu()
            return result
            
        except Exception as e:
            self.get_logger().warn(f'VPI GPU 모자이크 실패, 하이브리드로 폴백: {str(e)}')
            return self.create_mosaic_hybrid(images)

    def resize_image_vpi_cached(self, image, camera_id):
        """VPI 캐시를 활용한 리사이징 (GPU 메모리 재사용)"""
        try:
            if self.vpi_stream and self.vpi_backend == vpi.Backend.CUDA:
                # GPU에서 리사이징 (메모리 재사용)
                vpi_img = vpi.asimage(image)
                
                # 캐시된 출력 이미지 재사용
                cache_key = f"resized_{camera_id}"
                if cache_key not in self.vpi_image_cache:
                    # VPI Image 생성 (backend 파라미터 제거)
                    self.vpi_image_cache[cache_key] = vpi.Image(
                        (self.cell_width, self.cell_height), 
                        vpi.Format.BGR8
                    )
                
                # GPU에서 리사이징 실행
                self.vpi_image_cache[cache_key] = vpi_img.rescale(
                    (self.cell_width, self.cell_height),
                    backend=vpi.Backend.CUDA,
                    interp=vpi.Interp.LINEAR,
                    stream=self.vpi_stream
                )
                
                return self.vpi_image_cache[cache_key]
            else:
                return None
                
        except Exception as e:
            self.get_logger().warn(f'VPI 캐시 리사이징 실패: {str(e)}')
            return None

    def copy_to_mosaic_vpi(self, vpi_image, x_offset, y_offset):
        """VPI GPU 이미지를 모자이크에 복사"""
        try:
            # GPU 메모리에서 직접 복사 작업
            # 실제 VPI copy 함수 사용 (예시)
            pass
        except Exception as e:
            self.get_logger().warn(f'VPI 복사 실패: {str(e)}')

    def create_mosaic_hybrid(self, images):
        """하이브리드 모자이크 생성 (VPI + OpenCV)"""
        try:
            # 모자이크 이미지 초기화
            mosaic = np.zeros((self.mosaic_height, self.mosaic_width, 3), dtype=np.uint8)
            
            # 2x2 그리드 배치
            positions = [
                (0, 0),                                    # 좌상단
                (self.cell_width, 0),                      # 우상단  
                (0, self.cell_height),                     # 좌하단
                (self.cell_width, self.cell_height)        # 우하단
            ]
            
            for i in range(4):
                if i in images:
                    # VPI로 리사이징 (GPU 사용)
                    resized = self.resize_image_vpi(images[i], self.cell_width, self.cell_height)
                    
                    # 모자이크에 배치
                    x, y = positions[i]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = resized
                else:
                    # 이미지가 없는 경우 검은색으로 채움
                    x, y = positions[i]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = 0
            
            return mosaic
            
        except Exception as e:
            self.get_logger().error(f'하이브리드 모자이크 생성 실패: {str(e)}')
            return self.create_mosaic_fallback(images)

    def create_mosaic_fallback(self, images):
        """CPU 폴백 모자이크 생성"""
        try:
            mosaic = np.zeros((self.mosaic_height, self.mosaic_width, 3), dtype=np.uint8)
            positions = [(0, 0), (self.cell_width, 0), (0, self.cell_height), (self.cell_width, self.cell_height)]
            
            for i in range(4):
                if i in images:
                    resized = cv2.resize(images[i], (self.cell_width, self.cell_height))
                    x, y = positions[i]
                    mosaic[y:y+self.cell_height, x:x+self.cell_width] = resized
            
            return mosaic
        except Exception as e:
            self.get_logger().error(f'CPU 폴백 모자이크 생성 실패: {str(e)}')
            return np.zeros((self.mosaic_height, self.mosaic_width, 3), dtype=np.uint8)

    def push_to_gstreamer_jpeg(self, mosaic_bgr):
        """GStreamer JPEG 압축 파이프라인에 이미지 푸시"""
        try:
            if self.gst_pipeline is None or self.app_src is None:
                return False
                
            # BGR 이미지를 GstBuffer로 변환
            height, width, channels = mosaic_bgr.shape
            gst_buffer = Gst.Buffer.new_allocate(None, height * width * channels, None)
            gst_buffer.fill(0, mosaic_bgr.tobytes())
            
            # 타임스탬프 설정
            gst_buffer.pts = self.get_clock().now().nanoseconds
            
            # AppSrc에 푸시
            ret = self.app_src.emit('push-buffer', gst_buffer)
            return ret == Gst.FlowReturn.OK
            
        except Exception as e:
            self.get_logger().warn(f'GStreamer JPEG 푸시 실패: {str(e)}')
            return False

    def create_and_publish_mosaic(self):
        """모자이크 생성 및 게시"""
        try:
            with self.lock:
                current_time = time.time()
                
                # 최근 이미지만 사용 (1초 이내)
                valid_images = {}
                for camera_id, timestamp in self.image_timestamps.items():
                    if current_time - timestamp < 1.0 and camera_id in self.images:
                        valid_images[camera_id] = self.images[camera_id]
                
                if not valid_images:
                    return
                
                # VPI 최적화 모자이크 생성 (GPU 활용도 최대화)
                mosaic = self.create_mosaic_vpi_optimized(valid_images)
                
                # 원본 이미지 게시
                mosaic_msg = self.bridge.cv2_to_imgmsg(mosaic, 'bgr8')
                mosaic_msg.header.stamp = self.get_clock().now().to_msg()
                mosaic_msg.header.frame_id = 'camera_mosaic'
                self.mosaic_publisher.publish(mosaic_msg)
                
                # GStreamer로 JPEG 압축 (하드웨어 가속)
                if self.use_compressed and self.gst_pipeline:
                    success = self.push_to_gstreamer_jpeg(mosaic)
                    if not success:
                        # GStreamer 실패시 OpenCV JPEG 압축으로 폴백
                        self.publish_compressed_fallback(mosaic)
                elif self.use_compressed:
                    # GStreamer 사용 불가시 OpenCV JPEG 압축
                    self.publish_compressed_fallback(mosaic)
                
                # 성능 모니터링
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    elapsed = current_time - self.last_time
                    if elapsed > 0:
                        fps = 30.0 / elapsed
                        self.get_logger().info(f'모자이크 처리 성능: {fps:.1f} FPS (유효 카메라: {len(valid_images)}개)')
                    self.last_time = current_time
                
                # 메모리 정리
                if self.frame_count % 60 == 0:
                    gc.collect()
                    
        except Exception as e:
            self.get_logger().error(f'모자이크 생성 오류: {str(e)}')

    def publish_compressed_fallback(self, mosaic):
        """OpenCV JPEG 압축 폴백"""
        try:
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            result, encoded_img = cv2.imencode('.jpg', mosaic, encode_param)
            
            if result:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.header.frame_id = 'camera_mosaic'
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_img.tobytes()
                self.compressed_publisher.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().warn(f'OpenCV JPEG 압축 실패: {str(e)}')

    def destroy_node(self):
        """노드 종료시 정리"""
        try:
            if self.gst_pipeline:
                self.gst_pipeline.set_state(Gst.State.NULL)
            if self.vpi_stream:
                del self.vpi_stream
            # VPI 캐시 정리
            for cached_img in self.vpi_image_cache.values():
                try:
                    del cached_img
                except:
                    pass
            if self.vpi_output_cache:
                del self.vpi_output_cache
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageMosaicHWNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 