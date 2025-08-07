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

// GStreamer 파이프라인 템플릿
#define GST_PIPELINE_TEMPLATE_ECON_V4L2_BGRx \
	"v4l2src device=/dev/video%d io-mode=2 ! " \
	"video/x-raw,format=UYVY,width=%d,height=%d,framerate=60/1 ! " \
	"nvvidconv ! " \
	"video/x-raw(memory:NVMM),format=I420,width=%d,height=%d ! " \
	"nvvidconv nvbuf-memory-type=1 ! video/x-raw,format=BGRx ! " \
	"appsink name=sink sync=false emit-signals=true max-buffers=2 drop=true"

// 인코딩 타입 정의
typedef enum {
    GST_ENCODING_V4L2_BGRx,      // v4l2src → NVMM → BGRx (BGRA)
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

// 유틸리티 함수들
const char* gst_encoding_type_to_string(GstEncodingType type);

#endif // GST_ROS_PUBLISHER_H 