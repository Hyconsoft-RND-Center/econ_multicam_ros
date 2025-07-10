#include "econ_ros/gst_ros_publisher.h"

// 전역 변수 선언
static gboolean debug_mode = FALSE;

// 전방 선언 (v4l2src 기반 간소화)
static GstFlowReturn on_new_sample(GstElement *sink, GstRosPublisher *publisher);
static gboolean on_bus_message(GstBus *bus, GstMessage *msg, GstRosPublisher *publisher);
static void setup_appsink(GstRosPublisher *publisher);
static int setup_ros_messages(GstRosPublisher *publisher);
static void set_current_timestamp(builtin_interfaces__msg__Time *stamp);
static void publish_image_data(GstRosPublisher *publisher, 
                              unsigned char *data, 
                              int size, 
                              const char *encoding);

// 유틸리티 함수 구현
const char* gst_encoding_type_to_string(GstEncodingType type) {
    switch (type) {
        case GST_ENCODING_JPEG: return "JPEG";
        case GST_ENCODING_H264: return "H264";
        case GST_ENCODING_RAW: return "RAW";
        case GST_ENCODING_RGB: return "RGB";
        case GST_ENCODING_UYVY: return "UYVY";
        case GST_ENCODING_MULTI_STREAM: return "MULTI_STREAM";
        case GST_ENCODING_V4L2_RGB: return "V4L2_RGB";
        case GST_ENCODING_V4L2_I420: return "V4L2_I420";
        case GST_ENCODING_V4L2_UYVY: return "V4L2_UYVY";
        case GST_ENCODING_V4L2_JPEG: return "V4L2_JPEG";
        default: return "UNKNOWN";
    }
}

// 해상도에 따른 최적화된 설정 반환
// (삭제됨: 더 이상 사용되지 않음)

// 성능 모니터링 함수 (삭제됨)

// 설정 업데이트 함수 (삭제됨)

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
        case GST_ENCODING_JPEG:
            encoding = "rgb8";  // JPEG 디코딩 후 RGB 출력
            break;
        case GST_ENCODING_H264:
            encoding = "rgb8";  // H.264 디코딩 후 RGB 출력
            break;
        case GST_ENCODING_RAW:
        case GST_ENCODING_RGB:
            encoding = "rgb8";  // RGB 출력
            break;
        case GST_ENCODING_UYVY:
            encoding = "uyvy422";  // UYVY 그대로 출력
            break;
        case GST_ENCODING_MULTI_STREAM:
            encoding = "rgb8";  // 멀티 스트림도 RGB 출력
            break;
        // econ 권장 v4l2src 기반 파이프라인들
        case GST_ENCODING_V4L2_RGB:
            encoding = "bgra8";  // v4l2src → NVMM → BGRx (BGRA)
            break;
        case GST_ENCODING_V4L2_I420:
            encoding = "yuv420p";  // v4l2src → NVMM → I420
            break;
        case GST_ENCODING_V4L2_UYVY:
            encoding = "uyvy422";  // v4l2src → UYVY
            break;
        case GST_ENCODING_V4L2_JPEG:
            encoding = "jpeg";      // JPEG 비트스트림 (CompressedImage)
            break;
        default:
            encoding = "rgb8";
            break;
    }
    
    if (debug_mode && (callback_count <= 10 || callback_count % 100 == 0)) {
        printf("[DEBUG] cam %d: size %zu, enc %s\n",
               publisher->camera_id, map_info.size, encoding);
    }
    
    // 이미지 데이터 퍼블리시
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

// v4l2src 기반에서는 H.264 스트림이 불필요하므로 제거됨

// 버스 메시지 처리
static gboolean on_bus_message(GstBus *bus __attribute__((unused)), GstMessage *msg, GstRosPublisher *publisher) {
    GError *error = NULL;
    gchar *debug_info = NULL;
    
    if (!publisher) return FALSE;
    
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(msg, &error, &debug_info);
            printf("GStreamer Error from element %s: %s\n", 
                   GST_OBJECT_NAME(msg->src), error->message);
            if (debug_info) {
                printf("Debug info: %s\n", debug_info);
            }
            
            g_error_free(error);
            g_free(debug_info);
            
            // v4l2src 기반에서는 메인 루프가 불필요
            break;
            
        case GST_MESSAGE_WARNING:
            gst_message_parse_warning(msg, &error, &debug_info);
            printf("GStreamer Warning from element %s: %s\n", 
                   GST_OBJECT_NAME(msg->src), error->message);
            if (debug_info) {
                printf("Debug info: %s\n", debug_info);
            }
            
            g_error_free(error);
            g_free(debug_info);
            break;
            
        case GST_MESSAGE_INFO:
            if (debug_mode) {
                gst_message_parse_info(msg, &error, &debug_info);
                printf("GStreamer Info from element %s: %s\n", 
                       GST_OBJECT_NAME(msg->src), error->message);
                if (debug_info) {
                    printf("Debug info: %s\n", debug_info);
                }
                
                g_error_free(error);
                g_free(debug_info);
            }
            break;
            
        case GST_MESSAGE_EOS:
            printf("End-of-stream reached\n");
            // v4l2src 기반에서는 메인 루프가 불필요
            break;
            
        case GST_MESSAGE_STATE_CHANGED:
            if (debug_mode && GST_MESSAGE_SRC(msg) == GST_OBJECT(publisher->pipeline)) {
                GstState old_state, new_state, pending_state;
                gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
                printf("Pipeline state changed from %s to %s\n",
                       gst_element_state_get_name(old_state),
                       gst_element_state_get_name(new_state));
            }
            break;
            
        default:
            break;
    }
    
    return TRUE;
}

// v4l2src 기반에서는 스레드 함수들이 불필요하므로 제거됨

// v4l2src 기반에서는 appsrc 설정이 불필요하므로 제거됨

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
            case GST_ENCODING_UYVY:
            case GST_ENCODING_V4L2_UYVY:
                publisher->image_msg->step = publisher->width * 2; // UYVY = 2 bytes per pixel
                break;
            case GST_ENCODING_V4L2_I420:
                publisher->image_msg->step = publisher->width; // I420 Y plane = 1 byte per pixel
                break;
            default:
                publisher->image_msg->step = publisher->width * 3; // RGB = 3 bytes per pixel
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
    
    // v4l2src 기반에서는 H.264 멀티스트림이 불필요하므로 제거됨
    
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

        // ❶ 인코딩 문자열 최초 1회 설정
        if (publisher->image_msg->encoding.size == 0) {
            size_t encoding_len = strlen(encoding);
            publisher->image_msg->encoding.data = (char*)malloc(encoding_len + 1);
            if (publisher->image_msg->encoding.data) {
                strcpy(publisher->image_msg->encoding.data, encoding);
                publisher->image_msg->encoding.size = encoding_len;
                publisher->image_msg->encoding.capacity = encoding_len + 1;
            }
        }

        // ❷ 데이터 버퍼 재사용/확장
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
        
        // 카메라 정보 퍼블리시
        set_current_timestamp(&publisher->camera_info_msg->header.stamp);
        ret = rcl_publish(&publisher->camera_info_publisher, publisher->camera_info_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish camera info message: %s\n", rcl_get_error_string().str);
        }
    }
}

// GStreamer ROS 퍼블리시 생성 (v4l2src 기반 간소화된 버전)
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
    if (encoding_type == GST_ENCODING_V4L2_RGB) {
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_RGB, 
                 camera_id, width, height, width, height);
    } else if (encoding_type == GST_ENCODING_V4L2_I420) {
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_I420, 
                 camera_id, width, height, width, height);
    } else if (encoding_type == GST_ENCODING_V4L2_UYVY) {
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_UYVY, 
                 camera_id, width, height);
    } else if (encoding_type == GST_ENCODING_V4L2_JPEG) {
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_JPEG, 
                 camera_id, width, height, width, height);
    } else {
        printf("Unsupported encoding type for v4l2src: %s\n", gst_encoding_type_to_string(encoding_type));
        printf("Please use GST_ENCODING_V4L2_RGB, GST_ENCODING_V4L2_I420, or GST_ENCODING_V4L2_UYVY\n");
        pthread_mutex_destroy(&publisher->mutex);
        free(publisher);
        return NULL;
    }
    
    printf("Creating pipeline for camera %d (%s): %s\n", camera_id, gst_encoding_type_to_string(encoding_type), pipeline_str);
    
    // 파이프라인 생성
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
    if (encoding_type == GST_ENCODING_V4L2_JPEG) {
        publisher->publish_compressed = true;
        snprintf(topic_name, sizeof(topic_name), "/dev/video%d/image_raw/compressed", camera_id);
    } else {
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
    
    // 카메라 정보 퍼블리시 초기화
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

// GStreamer ROS 퍼블리시 시작 (v4l2src 기반 간소화된 버전)
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

// GStreamer ROS 퍼블리시 정지 (v4l2src 기반 간소화된 버전)
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

// v4l2src 기반에서는 프레임 푸시가 불필요하므로 제거됨
// v4l2src가 자동으로 카메라에서 데이터를 가져오기 때문

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
    
    // 뮤텍스 해제
    pthread_mutex_destroy(&publisher->mutex);
    
    // 메모리 해제
    free(publisher);
    
    printf("GStreamer ROS Publisher destroyed\n");
} 