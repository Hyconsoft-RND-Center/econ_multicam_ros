#include "econ_ros/gst_ros_publisher.h"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <pthread.h>
#include <rclc/rclc.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

// 전역 변수 선언
static gboolean debug_mode = FALSE;

// 전방 선언 (v4l2src 기반 간소화)
static GstFlowReturn on_new_sample(GstElement *sink, GstRosPublisher *publisher);

static void setup_appsink(GstRosPublisher *publisher);
static int setup_ros_messages(GstRosPublisher *publisher);
static void set_current_timestamp(builtin_interfaces__msg__Time *stamp);
static void publish_image_data(GstRosPublisher *publisher, 
                              unsigned char *data, 
                              int size, 
                              const char *encoding);

// VPI 3.2 color converter 사용

// 유틸리티 함수 구현
const char* gst_encoding_type_to_string(GstEncodingType type) {
    switch (type) {
        case GST_ENCODING_V4L2_BGRx: return "V4L2_BGRx";
        default: return "UNKNOWN";
    }
}

// 새로운 샘플 콜백 (JPEG/RAW용)
static GstFlowReturn on_new_sample(GstElement *sink, GstRosPublisher *publisher) {
    static unsigned long callback_count = 0;
    callback_count++;
    
    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map_info;
    
    if (!publisher) {
        printf("ERROR: appsink 콜백: publisher가 NULL\n");
        return GST_FLOW_ERROR;
    }
    
    if (debug_mode && (callback_count <= 10 || callback_count % 100 == 0)) {
        printf("[DEBUG] cam %d: appsink callback %lu\n",
               publisher->camera_id, callback_count);
    }
    
    // 샘플 가져오기
    sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {
        printf("ERROR: 카메라 %d: 샘플 가져오기 실패\n", publisher->camera_id);
        return GST_FLOW_ERROR;
    }
    
    buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        printf("ERROR: 카메라 %d: 버퍼 가져오기 실패\n", publisher->camera_id);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    
    // 버퍼 매핑
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
        printf("ERROR: 카메라 %d: 버퍼 매핑 실패\n", publisher->camera_id);
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    
    // 인코딩 타입에 따른 처리
    const char *encoding;
    switch (publisher->encoding_type) {
        case GST_ENCODING_V4L2_BGRx:
            encoding = "bgra8";  // v4l2src → NVMM → BGRx (BGRA)
            break;
    }
    
    if (debug_mode && (callback_count <= 10 || callback_count % 100 == 0)) {
        printf("[DEBUG] cam %d: size %zu, enc %s\n",
               publisher->camera_id, map_info.size, encoding);
    }
    
    // GStreamer 하드웨어 가속 방식: 원본 데이터 그대로 퍼블리시
    publish_image_data(publisher, map_info.data, map_info.size, encoding);
    
    // 통계 업데이트
    pthread_mutex_lock(&publisher->mutex);
    publisher->frame_count++;
    publisher->bytes_processed += map_info.size;
    pthread_mutex_unlock(&publisher->mutex);
    
    // 메모리 해제
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    
    if (debug_mode && (callback_count <= 10 || callback_count % 100 == 0)) {
        printf("[DEBUG] cam %d: callback done %lu\n",
               publisher->camera_id, callback_count);
    }
    
    return GST_FLOW_OK;
}

// appsink 설정
static void setup_appsink(GstRosPublisher *publisher) {
    g_object_set(G_OBJECT(publisher->sink),
                 "emit-signals", TRUE,
                 "sync", FALSE,
                 "max-buffers", 3,
                 "drop", TRUE,
                 "async", FALSE,
                 NULL);
    
    // 콜백 연결
    g_signal_connect(publisher->sink, "new-sample", 
                     G_CALLBACK(on_new_sample), publisher);
}

// ROS2 메시지 설정
static int setup_ros_messages(GstRosPublisher *publisher) {
    // CompressedImage 경로
    if (publisher->publish_compressed) {
        publisher->compressed_image_msg = (sensor_msgs__msg__CompressedImage*)malloc(sizeof(sensor_msgs__msg__CompressedImage));
        if (!publisher->compressed_image_msg) {
            printf("Failed to allocate CompressedImage message\n");
            return -1;
        }

        if (!sensor_msgs__msg__CompressedImage__init(publisher->compressed_image_msg)) {
            printf("Failed to initialize CompressedImage message\n");
            free(publisher->compressed_image_msg);
            return -1;
        }
        // format 고정 (jpeg)
        const char *fmt = "jpeg";
        size_t fmt_len = strlen(fmt);
        publisher->compressed_image_msg->format.data = (char*)malloc(fmt_len + 1);
        if (publisher->compressed_image_msg->format.data) {
            strcpy(publisher->compressed_image_msg->format.data, fmt);
            publisher->compressed_image_msg->format.size = fmt_len;
            publisher->compressed_image_msg->format.capacity = fmt_len + 1;
        }
    } else {
        // Image 메시지 초기화
        publisher->image_msg = (sensor_msgs__msg__Image*)malloc(sizeof(sensor_msgs__msg__Image));
        if (!publisher->image_msg) {
            printf("Failed to allocate image message\n");
            return -1;
        }

        if (!sensor_msgs__msg__Image__init(publisher->image_msg)) {
            printf("Failed to initialize image message\n");
            free(publisher->image_msg);
            return -1;
        }
    }

    // CameraInfo 메시지 초기화 (공통)
    publisher->camera_info_msg = (sensor_msgs__msg__CameraInfo*)malloc(sizeof(sensor_msgs__msg__CameraInfo));
    if (!publisher->camera_info_msg) {
        printf("Failed to allocate camera info message\n");
        // 해제
        if (publisher->publish_compressed) {
            sensor_msgs__msg__CompressedImage__fini(publisher->compressed_image_msg);
            free(publisher->compressed_image_msg);
        } else {
            sensor_msgs__msg__Image__fini(publisher->image_msg);
            free(publisher->image_msg);
        }
        return -1;
    }

    if (!sensor_msgs__msg__CameraInfo__init(publisher->camera_info_msg)) {
        printf("Failed to initialize camera info message\n");
        free(publisher->camera_info_msg);
        if (publisher->publish_compressed) {
            sensor_msgs__msg__CompressedImage__fini(publisher->compressed_image_msg);
            free(publisher->compressed_image_msg);
        } else {
            sensor_msgs__msg__Image__fini(publisher->image_msg);
            free(publisher->image_msg);
        }
        return -1;
    }
 
    // 기본 값 설정
    if (!publisher->publish_compressed) {
        publisher->image_msg->width = publisher->width;
        publisher->image_msg->height = publisher->height;
    }
 
    // 인코딩 타입에 따른 step 값 설정
    if (!publisher->publish_compressed) {
        switch (publisher->encoding_type) {
            case GST_ENCODING_V4L2_BGRx:
                publisher->image_msg->step = publisher->width * 4; // BGRA8 = 4 bytes per pixel
                break;
            default:
                publisher->image_msg->step = publisher->width * 4; // BGRA8 = 4 bytes per pixel
                break;
        }
        publisher->image_msg->is_bigendian = 0;
    }
 
    // Frame ID 설정
    char frame_id[32];
    snprintf(frame_id, sizeof(frame_id), "camera_%d", publisher->camera_id);
    
    size_t frame_id_len = strlen(frame_id);
    if (!publisher->publish_compressed) {
        publisher->image_msg->header.frame_id.data = (char*)malloc(frame_id_len + 1);
        if (publisher->image_msg->header.frame_id.data) {
            strcpy(publisher->image_msg->header.frame_id.data, frame_id);
            publisher->image_msg->header.frame_id.size = frame_id_len;
            publisher->image_msg->header.frame_id.capacity = frame_id_len + 1;
        }
    } else {
        publisher->compressed_image_msg->header.frame_id.data = (char*)malloc(frame_id_len + 1);
        if (publisher->compressed_image_msg->header.frame_id.data) {
            strcpy(publisher->compressed_image_msg->header.frame_id.data, frame_id);
            publisher->compressed_image_msg->header.frame_id.size = frame_id_len;
            publisher->compressed_image_msg->header.frame_id.capacity = frame_id_len + 1;
        }
    }
    
    // CameraInfo 설정
    publisher->camera_info_msg->width = publisher->width;
    publisher->camera_info_msg->height = publisher->height;
    
    publisher->camera_info_msg->header.frame_id.data = (char*)malloc(frame_id_len + 1);
    if (publisher->camera_info_msg->header.frame_id.data) {
        strcpy(publisher->camera_info_msg->header.frame_id.data, frame_id);
        publisher->camera_info_msg->header.frame_id.size = frame_id_len;
        publisher->camera_info_msg->header.frame_id.capacity = frame_id_len + 1;
    }
    
    return 0;
}

// 현재 시간 설정
static void set_current_timestamp(builtin_interfaces__msg__Time *stamp) {
    rcl_time_point_value_t now;
    if (rcutils_system_time_now(&now) == RCL_RET_OK) {
        stamp->sec = (int32_t)(now / 1000000000);
        stamp->nanosec = (uint32_t)(now % 1000000000);
    }
}

// 이미지 데이터 퍼블리시
static void publish_image_data(GstRosPublisher *publisher, 
                              unsigned char *data, 
                              int size, 
                              const char *encoding) {

    if (publisher->publish_compressed) {
        // CompressedImage 경로 --------------------------------------
        set_current_timestamp(&publisher->compressed_image_msg->header.stamp);

        // 데이터 버퍼 확장/재사용
        if (publisher->compressed_image_msg->data.capacity < (size_t)size) {
            uint8_t *new_buf = (uint8_t*)realloc(publisher->compressed_image_msg->data.data, size);
            if (!new_buf) {
                printf("Failed to realloc compressed buffer\n");
                return;
            }
            publisher->compressed_image_msg->data.data = new_buf;
            publisher->compressed_image_msg->data.capacity = size;
        }
        memcpy(publisher->compressed_image_msg->data.data, data, size);
        publisher->compressed_image_msg->data.size = size;

        rcl_ret_t ret = rcl_publish(&publisher->compressed_publisher, publisher->compressed_image_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish compressed image: %s\n", rcl_get_error_string().str);
        }

        // 카메라 정보 퍼블리시 (동일)
        set_current_timestamp(&publisher->camera_info_msg->header.stamp);
        ret = rcl_publish(&publisher->camera_info_publisher, publisher->camera_info_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish camera info message: %s\n", rcl_get_error_string().str);
        }

    } else {
        // 기존 Image 경로 -------------------------------------------
        set_current_timestamp(&publisher->image_msg->header.stamp);

        // 인코딩 문자열 최초 1회 설정
        if (publisher->image_msg->encoding.size == 0) {
            size_t encoding_len = strlen(encoding);
            publisher->image_msg->encoding.data = (char*)malloc(encoding_len + 1);
            if (publisher->image_msg->encoding.data) {
                strcpy(publisher->image_msg->encoding.data, encoding);
                publisher->image_msg->encoding.size = encoding_len;
                publisher->image_msg->encoding.capacity = encoding_len + 1;
            }
        }

        // 데이터 버퍼 재사용/확장
        if (publisher->image_msg->data.capacity < (size_t)size) {
            uint8_t *new_buf = (uint8_t*)realloc(publisher->image_msg->data.data, size);
            if (!new_buf) {
                printf("Failed to realloc image buffer\n");
                return;
            }
            publisher->image_msg->data.data = new_buf;
            publisher->image_msg->data.capacity = size;
        }
        memcpy(publisher->image_msg->data.data, data, size);
        publisher->image_msg->data.size = size;

        rcl_ret_t ret = rcl_publish(&publisher->image_publisher, publisher->image_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish image message: %s\n", rcl_get_error_string().str);
        }
        
        // camera_info 퍼블리시
        set_current_timestamp(&publisher->camera_info_msg->header.stamp);
        ret = rcl_publish(&publisher->camera_info_publisher, publisher->camera_info_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish camera info message: %s\n", rcl_get_error_string().str);
        }
    }
}

GstRosPublisher* gst_ros_publisher_create(int camera_id, int width, int height, 
                                         GstEncodingType encoding_type,
                                         rcl_node_t *node) {
    GstRosPublisher *publisher;
    char pipeline_str[2048];
    char topic_name[64];
    char camera_info_topic[80];
    
    // 메모리 할당
    publisher = (GstRosPublisher*)malloc(sizeof(GstRosPublisher));
    if (!publisher) {
        printf("Failed to allocate GstRosPublisher\n");
        return NULL;
    }
    
    memset(publisher, 0, sizeof(GstRosPublisher));
    
    // 기본 설정
    publisher->camera_id = camera_id;
    publisher->width = width;
    publisher->height = height;
    publisher->encoding_type = encoding_type;
    publisher->ros_node = node;
    publisher->is_running = 0;
    
    // 뮤텍스 초기화
    if (pthread_mutex_init(&publisher->mutex, NULL) != 0) {
        printf("Failed to initialize mutex\n");
        free(publisher);
        return NULL;
    }
    
    // GStreamer 초기화
    if (!gst_is_initialized()) {
        gst_init(NULL, NULL);
    }
    
    // 파이프라인 문자열 생성
    if (encoding_type == GST_ENCODING_V4L2_BGRx) {
            snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_BGRx, 
                     camera_id, width, height, width, height);
    } else {
        printf("Unsupported encoding type for v4l2src: %s\n", gst_encoding_type_to_string(encoding_type));
        printf("Please use GST_ENCODING_V4L2_BGRx\n");
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    printf("Creating pipeline for camera %d (%s): %s\n", camera_id, gst_encoding_type_to_string(encoding_type), pipeline_str);
    
    // 모든 경우에 파이프라인 생성
    GError *error = NULL;
    publisher->pipeline = gst_parse_launch(pipeline_str, &error);
    if (!publisher->pipeline) {
        printf("Failed to create GStreamer pipeline: %s\n", error ? error->message : "Unknown error");
        if (error) g_error_free(error);
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    // sink 엘리먼트 가져오기 (v4l2src는 설정 불필요)
    publisher->sink = gst_bin_get_by_name(GST_BIN(publisher->pipeline), "sink");
    if (!publisher->sink) {
        printf("Failed to get sink element\n");
        gst_object_unref(publisher->pipeline);
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    // appsink 설정
    setup_appsink(publisher);
    
    // ROS2 퍼블리시 설정
    if (encoding_type == GST_ENCODING_V4L2_BGRx) {
        publisher->publish_compressed = false;
        snprintf(topic_name, sizeof(topic_name), "/dev/video%d/image_raw", camera_id);
    }
    snprintf(camera_info_topic, sizeof(camera_info_topic), "/dev/video%d/camera_info", camera_id);
    
    // 퍼블리시 초기화
    const rosidl_message_type_support_t *image_type_support = NULL;
    if (publisher->publish_compressed) {
        image_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage);
    } else {
        image_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image);
    }
 
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    publisher_options.qos = rmw_qos_profile_sensor_data;
 
    rcl_ret_t ret;
    if (publisher->publish_compressed) {
        ret = rcl_publisher_init(&publisher->compressed_publisher, node,
                                 image_type_support, topic_name,
                                 &publisher_options);
    } else {
        ret = rcl_publisher_init(&publisher->image_publisher, node,
                                 image_type_support, topic_name,
                                 &publisher_options);
    }
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize image publisher: %s\n", rcl_get_error_string().str);
        gst_object_unref(publisher->sink);
        gst_object_unref(publisher->pipeline);
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    // camera_info 퍼블리시 초기화
    const rosidl_message_type_support_t *camera_info_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CameraInfo);
    
    rcl_publisher_options_t camera_info_options = rcl_publisher_get_default_options();
    camera_info_options.qos = rmw_qos_profile_sensor_data;
    
    ret = rcl_publisher_init(&publisher->camera_info_publisher, node, 
                             camera_info_type_support, camera_info_topic, 
                             &camera_info_options);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize camera info publisher: %s\n", rcl_get_error_string().str);
        if (publisher->publish_compressed) {
            rcl_publisher_fini(&publisher->compressed_publisher, node);
        } else {
            rcl_publisher_fini(&publisher->image_publisher, node);
        }
        gst_object_unref(publisher->sink);
        gst_object_unref(publisher->pipeline);
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    // ROS2 메시지 설정
    if (setup_ros_messages(publisher) != 0) {
        printf("Failed to setup ROS messages\n");
        rcl_publisher_fini(&publisher->image_publisher, node);
        rcl_publisher_fini(&publisher->camera_info_publisher, node);
        gst_object_unref(publisher->sink);
        gst_object_unref(publisher->pipeline);
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    printf("GStreamer ROS Publisher created successfully for camera %d\n", camera_id);
    return publisher;
}

// GStreamer ROS 퍼블리시 시작
int gst_ros_publisher_start(GstRosPublisher *publisher) {
    if (!publisher || publisher->is_running) {
        return -1;
    }
    
    // 파이프라인 시작
    GstStateChangeReturn ret = gst_element_set_state(publisher->pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        printf("Failed to start GStreamer pipeline for camera %d\n", publisher->camera_id);
        return -1;
    }
    
    // 상태 변경 대기 (최대 5초)
    ret = gst_element_get_state(publisher->pipeline, NULL, NULL, 5 * GST_SECOND);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        printf("Failed to reach PLAYING state for camera %d\n", publisher->camera_id);
        gst_element_set_state(publisher->pipeline, GST_STATE_NULL);
        return -1;
    }
    
    publisher->is_running = 1;
    
    printf("GStreamer ROS Publisher started for camera %d\n", publisher->camera_id);
    return 0;
}

// GStreamer ROS 퍼블리시 정지
int gst_ros_publisher_stop(GstRosPublisher *publisher) {
    if (!publisher || !publisher->is_running) {
        return -1;
    }
    
    publisher->is_running = 0;
    
    // 파이프라인 정지
    gst_element_set_state(publisher->pipeline, GST_STATE_NULL);
    
    printf("GStreamer ROS Publisher stopped for camera %d\n", publisher->camera_id);
    return 0;
}

// GStreamer ROS 퍼블리시 해제 (v4l2src 기반 간소화된 버전)
void gst_ros_publisher_destroy(GstRosPublisher *publisher) {
    if (!publisher) {
        return;
    }
    
    // 실행 중이면 정지
    if (publisher->is_running) {
        gst_ros_publisher_stop(publisher);
    }
    
    // ROS2 메시지 해제
    if (publisher->publish_compressed) {
        if (publisher->compressed_image_msg) {
            sensor_msgs__msg__CompressedImage__fini(publisher->compressed_image_msg);
            free(publisher->compressed_image_msg);
        }
    } else {
        if (publisher->image_msg) {
            sensor_msgs__msg__Image__fini(publisher->image_msg);
            free(publisher->image_msg);
        }
    }
    
    if (publisher->camera_info_msg) {
        sensor_msgs__msg__CameraInfo__fini(publisher->camera_info_msg);
        free(publisher->camera_info_msg);
    }
    
    // 퍼블리시 해제
    if (publisher->publish_compressed) {
        rcl_publisher_fini(&publisher->compressed_publisher, publisher->ros_node);
    } else {
        rcl_publisher_fini(&publisher->image_publisher, publisher->ros_node);
    }
    rcl_publisher_fini(&publisher->camera_info_publisher, publisher->ros_node);
    
    // GStreamer 리소스 해제
    if (publisher->sink) {
        gst_object_unref(publisher->sink);
    }

    if (publisher->pipeline) {
        gst_object_unref(publisher->pipeline);
    }
    
    // VPI support removed

    // 뮤텍스 해제
    pthread_mutex_destroy(&publisher->mutex);
    
    // 메모리 해제
    free(publisher);
    
    printf("GStreamer ROS Publisher destroyed\n");
} 