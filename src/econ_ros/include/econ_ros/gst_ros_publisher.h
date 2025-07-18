#ifndef GST_ROS_PUBLISHER_H
#define GST_ROS_PUBLISHER_H

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

// ROS2 관련 헤더
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/camera_info.h>
#include <builtin_interfaces/msg/time.h>
#include <rmw/types.h>
#include <rcutils/time.h>
#include <sensor_msgs/msg/compressed_image.h>
// VPI support removed

// GStreamer 파이프라인 템플릿들
#define GST_PIPELINE_TEMPLATE_NVARGUS \
	"nvarguscamerasrc sensor-id=%d ! " \
	"video/x-raw(memory:NVMM),width=%d,height=%d,framerate=30/1 ! " \
	"nvjpegenc ! " \
	"jpegdec ! " \
	"videoconvert ! " \
	"video/x-raw,format=RGB ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

#define GST_PIPELINE_TEMPLATE_JPEG \
	"gst-launch-1.0 v4l2src device=/dev/video%d ! " \
	"video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
	"v4l2jpegenc ! " \
	"jpegdec ! " \
	"videoconvert ! " \
	"video/x-raw,format=RGB ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// RGB 출력용 최적화된 파이프라인 (CPU 기반)
#define GST_PIPELINE_TEMPLATE_RGB_OPTIMIZED \
	"videoconvert ! video/x-raw,format=RGB ! "

// econ 권장 v4l2src 기반 파이프라인들

// VPI GPU template removed - use existing econ templates instead

// v4l2src → NVMM → BGRx (BGRA 호환)
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_RGB \
	"v4l2src device=/dev/video%d io-mode=2 ! " \
	"video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
	"nvvidconv ! " \
	"video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
	"nvvidconv nvbuf-memory-type=1 ! video/x-raw,format=BGRx ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// v4l2src → NVMM → I420 (효율적인 YUV420)
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_I420 \
	"v4l2src device=/dev/video%d io-mode=2 ! " \
	"video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
	"nvvidconv ! " \
	"video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// v4l2src → UYVY (가장 간단)
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_UYVY \
	"v4l2src device=/dev/video%d io-mode=2 ! " \
	"video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// v4l2src → JPEG hw encode/decode → RGB
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_JPEG \
	"v4l2src device=/dev/video%d io-mode=2 ! " \
	"video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
	"nvjpegenc ! " \
	"nvjpegdec ! " \
	"nvvidconv ! " \
	"video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
	"nvvidconv nvbuf-memory-type=1 ! video/x-raw,format=BGRx ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// 인코딩 타입 정의
typedef enum {
    GST_ENCODING_JPEG,
    GST_ENCODING_H264,
    GST_ENCODING_RAW,
    GST_ENCODING_RGB,           // 권장: GPU 가속 RGB 출력
    GST_ENCODING_UYVY,          // 디버깅용: UYVY 그대로 전송
    GST_ENCODING_MULTI_STREAM,
    // econ 권장 v4l2src 기반 파이프라인들
    GST_ENCODING_V4L2_RGB,      // v4l2src → NVMM → RGB (권장)
    GST_ENCODING_V4L2_I420,     // v4l2src → NVMM → I420 (효율적)
    GST_ENCODING_V4L2_UYVY,     // v4l2src → UYVY (가장 간단)
    GST_ENCODING_V4L2_JPEG      // v4l2src → JPEG hw encode/decode → RGB
    // VPI GPU encoding removed - use existing v4l2 encodings instead
} GstEncodingType;

// 파이프라인 성능 설정
typedef struct {
    int jpeg_quality;          // 0-100, JPEG 품질
    int h264_bitrate;          // H.264 비트레이트 (bps)
    int framerate;             // 프레임레이트
    int interpolation_method;  // nvvidconv 보간 방법 (0-5)
    int preset_level;          // 인코더 프리셋 레벨
    int max_buffers;           // 최대 버퍼 수
    int drop_frames;           // 프레임 드롭 활성화
} GstPipelineConfig;

// 기본 성능 설정
static const GstPipelineConfig DEFAULT_PIPELINE_CONFIG = {
    .jpeg_quality = 85,
    .h264_bitrate = 4000000,
    .framerate = 30,
    .interpolation_method = 5,  // Cubic interpolation
    .preset_level = 1,          // ultrafast
    .max_buffers = 1,
    .drop_frames = 1
};

// GStreamer ROS2 퍼블리시 구조체
typedef struct {
    // 기본 정보
    int camera_id;
    int width;
    int height;
    GstEncodingType encoding_type;
    
    // GStreamer 요소들
    GstElement *pipeline;
    GstElement *sink;           // appsink
    // appsrc는 v4l2src 기반에서 불필요하므로 제거
    
    // ROS2 퍼블리시 관련
    rcl_node_t* ros_node;
    rcl_publisher_t image_publisher;
    rcl_publisher_t camera_info_publisher;
    sensor_msgs__msg__Image* image_msg;
    sensor_msgs__msg__CameraInfo* camera_info_msg;
    // JPEG 압축 전송용
    rcl_publisher_t compressed_publisher;
    sensor_msgs__msg__CompressedImage* compressed_image_msg;
    bool publish_compressed;
    
    // 스레드 동기화
    pthread_mutex_t mutex;
    
    // 통계 정보
    unsigned long frame_count;
    unsigned long bytes_processed;
    
    // 상태 관리
    int is_running;
} GstRosPublisher;

// 함수 선언들
GstRosPublisher* gst_ros_publisher_create(
    int camera_id, 
    int width, 
    int height, 
    GstEncodingType encoding_type,
    rcl_node_t* ros_node
);

int gst_ros_publisher_start(GstRosPublisher* publisher);
int gst_ros_publisher_stop(GstRosPublisher* publisher);
void gst_ros_publisher_destroy(GstRosPublisher* publisher);

// v4l2src 기반에서는 프레임 푸시 함수 불필요
// 다음 함수들 제거:
// - gst_ros_publisher_push_frame()
// - gst_ros_publisher_push_buffer()

// 유틸리티 함수들
const char* gst_encoding_type_to_string(GstEncodingType type);

// JetPack 6.2.0 optimized nvvidconv configurations
#define GST_PIPELINE_TEMPLATE_JETPACK62_OPTIMIZED \
    "v4l2src device=/dev/video%d io-mode=mmap ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=%d/1 ! " \
    "nvvidconv nvbuf-memory-type=0 compute-hw=1 interpolation-method=1 ! " \
    "video/x-raw(memory:NVMM),format=BGRx ! " \
    "nvvidconv nvbuf-memory-type=1 ! " \
    "video/x-raw,format=BGRx ! " \
    "appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true"

#define GST_PIPELINE_TEMPLATE_JETPACK62_FALLBACK \
    "v4l2src device=/dev/video%d io-mode=mmap ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=%d/1 ! " \
    "nvvidconv ! " \
    "video/x-raw(memory:NVMM),format=BGRx ! " \
    "nvvidconv ! " \
    "video/x-raw,format=BGRx ! " \
    "appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true"

#endif // GST_ROS_PUBLISHER_H 