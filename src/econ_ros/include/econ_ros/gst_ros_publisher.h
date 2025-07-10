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

// Jetson GPU 가속 파이프라인 정의 - 최적화된 버전
#define GST_PIPELINE_TEMPLATE_JPEG_OPTIMIZED \
    "appsrc name=source format=time ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
    "nvvidconv interpolation-method=5 ! " \
    "video/x-raw(memory:NVMM),format=I420 ! " \
    "nvjpegenc quality=85 ! " \
    "nvjpegdec ! " \
    "nvvidconv ! " \
    "video/x-raw,format=RGB ! " \
    "appsink name=sink sync=false emit-signals=true max-buffers=1 drop=true"

#define GST_PIPELINE_TEMPLATE_H264_OPTIMIZED \
    "appsrc name=source format=time ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
    "nvvidconv interpolation-method=5 ! " \
    "video/x-raw(memory:NVMM),format=I420 ! " \
    "nvv4l2h264enc bitrate=4000000 profile=baseline level=4.0 preset-level=ultrafast ! " \
    "h264parse ! " \
    "nvv4l2decoder ! " \
    "nvvidconv ! " \
    "video/x-raw,format=RGB ! " \
    "appsink name=sink sync=false emit-signals=true max-buffers=1 drop=true"

// 추가 파이프라인 - RAW 포맷 (디버깅용)
#define GST_PIPELINE_TEMPLATE_RAW_OPTIMIZED \
    "appsrc name=source format=time ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
    "nvvidconv interpolation-method=5 ! " \
    "video/x-raw,format=RGB ! " \
    "appsink name=sink sync=false emit-signals=true max-buffers=1 drop=true"

// 고성능 RGB 파이프라인 (권장)
#define GST_PIPELINE_TEMPLATE_RGB_OPTIMIZED \
    "appsrc name=source ! " \
    "queue max-size-buffers=2 ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
    "videoconvert ! " \
    "video/x-raw,format=RGB ! " \
    "queue max-size-buffers=2 ! " \
    "appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// 추가: 디버깅용 간단한 파이프라인 (UYVY 그대로 전송)
#define GST_PIPELINE_TEMPLATE_UYVY_SIMPLE \
    "appsrc name=source ! " \
    "queue max-size-buffers=2 ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
    "appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// 멀티 스트림 파이프라인 - 동시에 JPEG와 H.264 출력
#define GST_PIPELINE_TEMPLATE_MULTI_STREAM \
    "appsrc name=source format=time ! " \
    "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
    "nvvidconv interpolation-method=5 ! " \
    "video/x-raw(memory:NVMM),format=I420 ! " \
    "tee name=t ! " \
    "queue ! nvjpegenc quality=85 ! nvjpegdec ! nvvidconv ! video/x-raw,format=RGB ! " \
    "appsink name=sink-jpeg sync=false emit-signals=true max-buffers=1 drop=true " \
    "t. ! queue ! nvv4l2h264enc bitrate=4000000 profile=baseline preset-level=ultrafast ! " \
    "h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGB ! " \
    "appsink name=sink-h264 sync=false emit-signals=true max-buffers=1 drop=true"

/* RGB */
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_RGB \
  "v4l2src device=/dev/video%d io-mode=2 ! " \
  "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
  "nvvidconv ! " \
  "video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
  "nvvidconv nvbuf-memory-type=1 ! video/x-raw,format=BGRx ! " \
  "appsink name=sink sync=false emit-signals=true max-buffers=4 drop=true"

/* I420 */
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_I420 \
  "v4l2src device=/dev/video%d io-mode=2 ! " \
  "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
  "nvvidconv ! " \
  "video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
  "nvvidconv nvbuf-memory-type=1 ! video/x-raw,format=I420 ! " \
  "appsink name=sink sync=false emit-signals=true max-buffers=4 drop=true"

/* UYVY */
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_UYVY \
  "v4l2src device=/dev/video%d io-mode=2 ! " \
  "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
  "nvvidconv nvbuf-memory-type=1 ! video/x-raw,format=UYVY ! " \
  "appsink name=sink sync=false emit-signals=true max-buffers=4 drop=true"

/* JPEG (encode only, publish compressed bitstream) */
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_JPEG \
  "v4l2src device=/dev/video%d io-mode=2 ! " \
  "video/x-raw,format=UYVY,width=%d,height=%d,framerate=30/1 ! " \
  "nvvidconv ! " \
  "video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
  "nvjpegenc quality=85 ! " \
  "appsink name=sink sync=false emit-signals=true max-buffers=4 drop=true"

// 백워드 호환성을 위한 기존 파이프라인 유지
#define GST_PIPELINE_TEMPLATE_JPEG GST_PIPELINE_TEMPLATE_JPEG_OPTIMIZED
#define GST_PIPELINE_TEMPLATE_H264 GST_PIPELINE_TEMPLATE_H264_OPTIMIZED

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
void gst_ros_publisher_print_stats(GstRosPublisher* publisher);

#endif // GST_ROS_PUBLISHER_H 