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
static void* processing_thread_func(void* data);

static void setup_appsink(GstRosPublisher *publisher);
static int setup_ros_messages(GstRosPublisher *publisher);
static void set_current_timestamp(builtin_interfaces__msg__Time *stamp);
static void publish_image_data(GstRosPublisher *publisher, 
                              unsigned char *data, 
                              int size, 
                              const char *encoding,
                              GstBuffer *buffer);

// VPI 3.2 color converter 사용

// VPI GPU 처리 함수들 (조건부 컴파일)
#if HAVE_VPI
static int init_vpi_gpu_processing(GstRosPublisher *publisher);
static void cleanup_vpi_gpu_processing(GstRosPublisher *publisher);
static int ExtractFdFromNvBuffer(void *nvbuf_surface);
static int process_nvmm_to_bgra_gpu(GstRosPublisher *publisher, GstBuffer *buffer, unsigned char **output_data, int *output_size);
#endif

// 유틸리티 함수 구현
const char* gst_encoding_type_to_string(GstEncodingType type) {
    switch (type) {
        case GST_ENCODING_V4L2_BGRx: return "V4L2_BGRx";
        case GST_ENCODING_V4L2_NVMM_JPEG: return "V4L2_NVMM_JPEG";
        case GST_ENCODING_V4L2_VPI_GPU: return "V4L2_VPI_GPU";
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
        case GST_ENCODING_V4L2_NVMM_JPEG:
            encoding = "jpeg";   // v4l2src → NVMM → JPEG
            break;
        case GST_ENCODING_V4L2_VPI_GPU:
            encoding = "bgra8";  // VPI GPU → BGRA8
            break;
        default:
            encoding = "unknown";
            break;
    }
    
    if (debug_mode && (callback_count <= 10 || callback_count % 100 == 0)) {
        printf("[DEBUG] cam %d: size %zu, enc %s\n",
               publisher->camera_id, map_info.size, encoding);
    }
    
    // Zero-copy 최적화를 위해 GStreamer 버퍼 레퍼런스 증가
    gst_buffer_ref(buffer);
    
    // 메모리 복사 최적화: zero-copy 방식 사용
    publish_image_data(publisher, map_info.data, map_info.size, encoding, buffer);
    
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

// 스레드 기반 처리 함수 (emit-signals=false 최적화)
static void* processing_thread_func(void* data) {
    GstRosPublisher *publisher = (GstRosPublisher*)data;
    GstAppSink *appsink = GST_APP_SINK(publisher->sink);
    unsigned long callback_count = 0;
    
    printf("[INFO] Processing thread started for camera %d\n", publisher->camera_id);
    
    while (publisher->is_running) {
        // 50ms 타임아웃으로 샘플 시도
        GstSample *sample = gst_app_sink_try_pull_sample(appsink, 50000000); // 50ms in nanoseconds
        if (!sample) {
            continue; // 타임아웃 → 다음 루프
        }
        
        callback_count++;
        
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            printf("ERROR: 카메라 %d: 버퍼 가져오기 실패\n", publisher->camera_id);
            gst_sample_unref(sample);
            continue;
        }
        
        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
            printf("ERROR: 카메라 %d: 버퍼 매핑 실패\n", publisher->camera_id);
            gst_sample_unref(sample);
            continue;
        }
        
        // 인코딩 타입에 따른 처리
        const char *encoding;
        switch (publisher->encoding_type) {
            case GST_ENCODING_V4L2_BGRx:
                encoding = "bgra8";
                break;
            case GST_ENCODING_V4L2_NVMM_JPEG:
                encoding = "jpeg";
                break;
            case GST_ENCODING_V4L2_VPI_GPU:
                encoding = "bgra8";  // VPI GPU로 BGRA8 변환
                break;
            default:
                encoding = "unknown";
                break;
        }
        
        if (debug_mode && (callback_count <= 10 || callback_count % 100 == 0)) {
            printf("[DEBUG] cam %d: thread processing %lu, size %zu\n",
                   publisher->camera_id, callback_count, map_info.size);
        }
        
        // VPI GPU 처리 또는 일반 처리
        if (publisher->use_vpi_gpu_processing) {
#if HAVE_VPI
            // VPI GPU 처리 임시 비활성화 - 시스템 안정성 문제로 주석 처리
            printf("VPI GPU processing temporarily disabled due to system stability issues\n");
            printf("Falling back to standard CPU processing\n");
            gst_buffer_ref(buffer);
            publish_image_data(publisher, map_info.data, map_info.size, encoding, buffer);
            
            /*
            // VPI GPU로 NVMM I420 → BGRA8 변환 (안정성 문제로 주석 처리)
            unsigned char *gpu_output_data;
            int gpu_output_size;
            
            if (process_nvmm_to_bgra_gpu(publisher, buffer, &gpu_output_data, &gpu_output_size) == 0) {
                // GPU 처리된 데이터로 퍼블리시
                gst_buffer_ref(buffer);
                publish_image_data(publisher, gpu_output_data, gpu_output_size, encoding, buffer);
            } else {
                printf("Failed to process NVMM buffer with VPI GPU\n");
            }
            */
#else
            printf("VPI GPU processing requested but VPI not available - falling back to default\n");
            gst_buffer_ref(buffer);
            publish_image_data(publisher, map_info.data, map_info.size, encoding, buffer);
#endif
        } else {
            // 기존 방식: Zero-copy 최적화
            gst_buffer_ref(buffer);
            publish_image_data(publisher, map_info.data, map_info.size, encoding, buffer);
        }
        
        // 통계 업데이트
        pthread_mutex_lock(&publisher->mutex);
        publisher->frame_count++;
        publisher->bytes_processed += map_info.size;
        pthread_mutex_unlock(&publisher->mutex);
        
        // 메모리 해제
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
    }
    
    printf("[INFO] Processing thread stopped for camera %d\n", publisher->camera_id);
    return NULL;
}

// appsink 설정
static void setup_appsink(GstRosPublisher *publisher) {
    if (publisher->use_thread_processing) {
        // 스레드 기반 처리: emit-signals=false 로 최적화
        g_object_set(G_OBJECT(publisher->sink),
                     "emit-signals", FALSE,
                     "sync", FALSE,
                     "max-buffers", 2,
                     "drop", TRUE,
                     "async", FALSE,
                     "enable-last-sample", FALSE,
                     NULL);
    } else {
        // 기존 콜백 방식
        g_object_set(G_OBJECT(publisher->sink),
                     "emit-signals", TRUE,
                     "sync", FALSE,
                     "max-buffers", 2,
                     "drop", TRUE,
                     "async", FALSE,
                     NULL);
        
        // 콜백 연결
        g_signal_connect(publisher->sink, "new-sample", 
                         G_CALLBACK(on_new_sample), publisher);
    }
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
        
        // Zero-copy 최적화를 위해 초기 data 버퍼는 NULL로 설정
        publisher->image_msg->data.data = NULL;
        publisher->image_msg->data.size = 0;
        publisher->image_msg->data.capacity = 0;
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
            case GST_ENCODING_V4L2_VPI_GPU:
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
                              const char *encoding,
                              GstBuffer *buffer) {

    if (publisher->publish_compressed) {
        // CompressedImage 경로 --------------------------------------
        set_current_timestamp(&publisher->compressed_image_msg->header.stamp);

        // NVMM JPEG 경로: Zero-copy 최적화
        if (strcmp(encoding, "jpeg") == 0) {
            // 기존 할당된 버퍼가 있다면 해제 (zero-copy가 아닌 경우)
            if (publisher->compressed_image_msg->data.data && publisher->compressed_image_msg->data.capacity > 0) {
                free(publisher->compressed_image_msg->data.data);
            }
            
            // Zero-copy: GStreamer 버퍼 데이터 포인터 직접 사용
            publisher->compressed_image_msg->data.data = data;
            publisher->compressed_image_msg->data.size = size;
            publisher->compressed_image_msg->data.capacity = 0; // zero-copy 마커
        } else {
            // 기존 방식: 메모리 복사
            if (publisher->compressed_image_msg->data.capacity < (size_t)size) {
                uint8_t *new_buf = (uint8_t*)realloc(publisher->compressed_image_msg->data.data, size);
                if (!new_buf) {
                    printf("Failed to realloc compressed buffer\n");
                    gst_buffer_unref(buffer);
                    return;
                }
                publisher->compressed_image_msg->data.data = new_buf;
                publisher->compressed_image_msg->data.capacity = size;
            }
            memcpy(publisher->compressed_image_msg->data.data, data, size);
            publisher->compressed_image_msg->data.size = size;
        }

        rcl_ret_t ret = rcl_publish(&publisher->compressed_publisher, publisher->compressed_image_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish compressed image: %s\n", rcl_get_error_string().str);
        }
        
        // JPEG zero-copy 경우: 퍼블리시 완료 후 GStreamer 버퍼 레퍼런스 해제
        if (strcmp(encoding, "jpeg") == 0 && publisher->compressed_image_msg->data.capacity == 0) {
            // 이전 버퍼 레퍼런스 해제
            if (publisher->current_gst_buffer) {
                gst_buffer_unref(publisher->current_gst_buffer);
            }
            // 새로운 버퍼 레퍼런스 저장
            publisher->current_gst_buffer = buffer;
        } else {
            // 기존 방식: 즉시 해제
            gst_buffer_unref(buffer);
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

        // Zero-copy 최적화: BGRA8 Raw 경로에서 포인터 직접 할당
        if (strcmp(encoding, "bgra8") == 0) {
            // 기존 할당된 버퍼가 있다면 해제 (zero-copy가 아닌 경우)
            if (publisher->image_msg->data.data && publisher->image_msg->data.capacity > 0) {
                free(publisher->image_msg->data.data);
            }
            
            // Zero-copy: GStreamer 버퍼 데이터 포인터 직접 사용
            publisher->image_msg->data.data = data;
            publisher->image_msg->data.size = size;
            publisher->image_msg->data.capacity = 0; // zero-copy 마커 (free 하지 않음)
            
            // 이전 GStreamer 버퍼 레퍼런스 해제
            if (publisher->current_gst_buffer) {
                gst_buffer_unref(publisher->current_gst_buffer);
            }
            // 새로운 GStreamer 버퍼 레퍼런스 저장
            publisher->current_gst_buffer = buffer;
        } else {
            // 다른 인코딩의 경우 기존 메모리 복사 방식 유지
            if (publisher->image_msg->data.capacity < (size_t)size) {
                if (publisher->image_msg->data.data) {
                    free(publisher->image_msg->data.data);
                }
                publisher->image_msg->data.data = (uint8_t*)malloc(size);
                if (!publisher->image_msg->data.data) {
                    printf("Failed to allocate image buffer\n");
                    gst_buffer_unref(buffer); // 에러 시 버퍼 레퍼런스 해제
                    return;
                }
                publisher->image_msg->data.capacity = size;
            }
            memcpy(publisher->image_msg->data.data, data, size);
            publisher->image_msg->data.size = size;
            gst_buffer_unref(buffer); // 복사 완료 후 버퍼 레퍼런스 해제
        }

        rcl_ret_t ret = rcl_publish(&publisher->image_publisher, publisher->image_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish image message: %s\n", rcl_get_error_string().str);
        }
        
        // Zero-copy 경우: 퍼블리시 완료 후 GStreamer 버퍼 레퍼런스 해제 (수정된 조건)
        if (strcmp(encoding, "bgra8") == 0) {
            if (publisher->current_gst_buffer) {
                gst_buffer_unref(publisher->current_gst_buffer);
                publisher->current_gst_buffer = NULL;
            }
        }
        
        // camera_info 퍼블리시
        set_current_timestamp(&publisher->camera_info_msg->header.stamp);
        ret = rcl_publish(&publisher->camera_info_publisher, publisher->camera_info_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish camera info message: %s\n", rcl_get_error_string().str);
        }
    }
}

#if HAVE_VPI
// VPI 3.2 GPU 처리 초기화 함수
static int init_vpi_gpu_processing(GstRosPublisher *publisher) {
    VPIStatus status;
    
    // VPI Context 생성 (CUDA 백엔드 활성화)
    status = vpiContextCreate(VPI_BACKEND_CUDA, &publisher->vpi_ctx);
    if (status != VPI_SUCCESS) {
        printf("Failed to create VPI context: %d\n", status);
        return -1;
    }
    
    // VPI Context 설정
    status = vpiContextSetCurrent(publisher->vpi_ctx);
    if (status != VPI_SUCCESS) {
        printf("Failed to set VPI context: %d\n", status);
        vpiContextDestroy(publisher->vpi_ctx);
        return -1;
    }
    
    // VPI Stream 생성 (CUDA 백엔드)
    status = vpiStreamCreate(VPI_BACKEND_CUDA, &publisher->vpi_stream);
    if (status != VPI_SUCCESS) {
        printf("Failed to create VPI stream: %d\n", status);
        vpiContextDestroy(publisher->vpi_ctx);
        return -1;
    }
    
    // VPI 출력 이미지 생성 (BGRA8 format)
    status = vpiImageCreate(publisher->width, publisher->height, VPI_IMAGE_FORMAT_BGRA8, 
                           VPI_BACKEND_CUDA, &publisher->vpi_output_image);
    if (status != VPI_SUCCESS) {
        printf("Failed to create VPI output image: %d\n", status);
        vpiStreamDestroy(publisher->vpi_stream);
        vpiContextDestroy(publisher->vpi_ctx);
        return -1;
    }
    
    printf("VPI 3.2 GPU processing initialized for camera %d\n", publisher->camera_id);
    return 0;
}

// VPI 3.2 GPU 처리 정리 함수
static void cleanup_vpi_gpu_processing(GstRosPublisher *publisher) {
    if (publisher->vpi_input_image) {
        vpiImageDestroy(publisher->vpi_input_image);
        publisher->vpi_input_image = NULL;
    }
    if (publisher->vpi_output_image) {
        vpiImageDestroy(publisher->vpi_output_image);
        publisher->vpi_output_image = NULL;
    }
    if (publisher->vpi_stream) {
        vpiStreamDestroy(publisher->vpi_stream);
        publisher->vpi_stream = NULL;
    }
    if (publisher->vpi_ctx) {
        vpiContextDestroy(publisher->vpi_ctx);
        publisher->vpi_ctx = NULL;
    }
}

// NvBufSurface에서 dmabuf FD 추출 (VPI 공식 방법)
static int ExtractFdFromNvBuffer(void *nvbuf_surface) {
    NvBufSurface *surface = (NvBufSurface*)nvbuf_surface;
    if (!surface || surface->numFilled == 0) {
        return -1;
    }
    
    // NvBufSurface의 첫 번째 버퍼에서 dmabuf fd 추출
    // NVIDIA 공식 방법: NvBufSurfaceMemMap 후 fd 접근
    return surface->surfaceList[0].bufferDesc;
}

// NVMM I420 → VPI CUDA → BGRA8 변환 함수
static int process_nvmm_to_bgra_gpu(GstRosPublisher *publisher, GstBuffer *buffer, 
                                    unsigned char **output_data, int *output_size) {
    // NVIDIA 공식 VPI YUV420 호환성 해결 방법: NvBuffer 변환 활용
    printf("Starting VPI GPU processing using NVIDIA's official NvBuffer workaround\n");
    
    VPIStatus status;
    
    // GStreamer NVMM 버퍼를 NvBufSurface로 변환
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
        printf("Failed to map GStreamer buffer\n");
        return -1;
    }
    
    NvBufSurface *nvbuf_surface = (NvBufSurface *)map_info.data;
    if (!nvbuf_surface) {
        printf("Failed to cast mapped data to NvBufSurface\n");
        gst_buffer_unmap(buffer, &map_info);
        return -1;
    }
    
    printf("NvBufSurface: numFilled=%d, memType=%d, colorFormat=%d\n", 
           nvbuf_surface->numFilled, nvbuf_surface->memType, 
           nvbuf_surface->surfaceList[0].colorFormat);
    
    // VPI 컨텍스트 설정
    status = vpiContextSetCurrent(publisher->vpi_ctx);
    if (status != VPI_SUCCESS) {
        char error_msg[256];
        vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
        printf("Failed to set VPI context: %d (%s)\n", status, error_msg);
        gst_buffer_unmap(buffer, &map_info);
        return -1;
    }
    
    // 핵심: NVIDIA 공식 VPI NvBuffer 워크어라운드 구현
    // "A workaround is to use NvBuffer (iNativeBuffer->createNvBuffer() & iNativeBuffer->copyToNvBuffer())"
    
    VPIImage input_vpi_image = NULL;
    
    // 방법 1: vpiImageCreateWrapper() 사용 - NvBuffer dmabuf_fd를 그대로 사용
    printf("Attempting VPI image creation from NvBuffer dmabuf wrapper...\n");
    
    // NvBufSurface에서 dmabuf_fd 추출
    int dmabuf_fd = ExtractFdFromNvBuffer((void*)nvbuf_surface);
    printf("Extracted dmabuf_fd: %d from NvBufSurface\n", dmabuf_fd);
    
    int format_success = 0;
    
    if (dmabuf_fd >= 0) {
        // VPIImageData 구조체 설정 - NvBuffer를 그대로 래핑
        VPIImageData image_data = {0};
        image_data.bufferType = VPI_IMAGE_BUFFER_NVBUFFER;
        image_data.buffer.fd = dmabuf_fd;
        
        // NvBuffer를 VPI로 래핑 (원본 NV12 포맷 유지)
        status = vpiImageCreateWrapper(&image_data, NULL, VPI_BACKEND_CUDA, &input_vpi_image);
        if (status == VPI_SUCCESS) {
            printf("Successfully wrapped NvBuffer dmabuf_fd: %d as VPI image\n", dmabuf_fd);
            format_success = 1;
        } else {
            char error_msg[256];
            vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
            printf("NvBuffer wrapping failed: %s\n", error_msg);
        }
    } else {
        printf("Failed to extract dmabuf_fd from NvBufSurface\n");
    }
    
    // 방법 2: 직접 변환이 실패한 경우 - CUDA 기반 수동 변환
    if (!format_success) {
        printf("Direct NvBuffer→VPI conversion failed. Using CUDA-based manual conversion...\n");
        
        // NVMM 버퍼 매핑
        if (NvBufSurfaceMap(nvbuf_surface, 0, 0, NVBUF_MAP_READ_WRITE) != 0) {
            printf("NvBufSurfaceMap failed\n");
            gst_buffer_unmap(buffer, &map_info);
            return -1;
        }
        
        if (NvBufSurfaceSyncForDevice(nvbuf_surface, 0, 0) != 0) {
            printf("NvBufSurfaceSyncForDevice failed\n");
            NvBufSurfaceUnMap(nvbuf_surface, 0, 0);
            gst_buffer_unmap(buffer, &map_info);
            return -1;
        }
        
        // VPI 호환 RGB 이미지 직접 생성
        status = vpiImageCreate(publisher->width, publisher->height, VPI_IMAGE_FORMAT_BGR8, 
                              VPI_BACKEND_CUDA, &input_vpi_image);
        if (status != VPI_SUCCESS) {
            char error_msg[256];
            vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
            printf("Failed to create VPI BGR8 image: %d (%s)\n", status, error_msg);
            NvBufSurfaceUnMap(nvbuf_surface, 0, 0);
            gst_buffer_unmap(buffer, &map_info);
            return -1;
        }
        
        // 수동 NV12 → BGR 변환
        VPIImageData vpi_bgr_data;
        status = vpiImageLockData(input_vpi_image, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR, &vpi_bgr_data);
        if (status != VPI_SUCCESS) {
            char error_msg[256];
            vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
            printf("Failed to lock VPI BGR image: %d (%s)\n", status, error_msg);
            vpiImageDestroy(input_vpi_image);
            NvBufSurfaceUnMap(nvbuf_surface, 0, 0);
            gst_buffer_unmap(buffer, &map_info);
            return -1;
        }
        
        // 간소화된 NV12 → BGR 변환 (Y 채널을 흑백 RGB로)
        void *nvmm_y_ptr = nvbuf_surface->surfaceList[0].mappedAddr.addr[0];
        printf("Manual conversion: Y=%p → BGR=%p (size: %dx%d)\n", 
               nvmm_y_ptr, vpi_bgr_data.buffer.pitch.planes[0].data,
               publisher->width, publisher->height);
        
        // CUDA 메모리 직접 복사로 Y→BGR 변환
        unsigned char *src_y = (unsigned char*)nvmm_y_ptr;
        unsigned char *dst_bgr = (unsigned char*)vpi_bgr_data.buffer.pitch.planes[0].data;
        
        for (int row = 0; row < publisher->height; row++) {
            unsigned char *src_row = src_y + row * nvbuf_surface->surfaceList[0].pitch;
            unsigned char *dst_row = dst_bgr + row * vpi_bgr_data.buffer.pitch.planes[0].pitchBytes;
            
            for (int col = 0; col < publisher->width; col++) {
                unsigned char y_val = src_row[col];
                // BGR 포맷으로 흑백 값 저장
                dst_row[col * 3 + 0] = y_val; // B
                dst_row[col * 3 + 1] = y_val; // G  
                dst_row[col * 3 + 2] = y_val; // R
            }
        }
        
        vpiImageUnlock(input_vpi_image);
        NvBufSurfaceUnMap(nvbuf_surface, 0, 0);
        
        printf("Manual NV12→BGR conversion completed successfully\n");
    }
    
    // 3. VPI를 사용한 GPU 가속 색상 처리
    // BGR8 → BGRA8 변환 (알파 채널 추가)
    status = vpiSubmitConvertImageFormat(publisher->vpi_stream, VPI_BACKEND_CUDA,
                                        input_vpi_image, publisher->vpi_output_image, NULL);
    if (status != VPI_SUCCESS) {
        char error_msg[256];
        vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
        printf("Failed to submit VPI format conversion: %d (%s)\n", status, error_msg);
        vpiImageDestroy(input_vpi_image);
        gst_buffer_unmap(buffer, &map_info);
        return -1;
    }

    // VPI 스트림 동기화 (GPU 처리 완료 대기)
    status = vpiStreamSync(publisher->vpi_stream);
    if (status != VPI_SUCCESS) {
        char error_msg[256];
        vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
        printf("Failed to sync VPI stream: %d (%s)\n", status, error_msg);
        vpiImageDestroy(input_vpi_image);
        gst_buffer_unmap(buffer, &map_info);
        return -1;
    }

    // 4. 출력 데이터 추출
    VPIImageData output_img_data;
    status = vpiImageLockData(publisher->vpi_output_image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_CUDA_PITCH_LINEAR, &output_img_data);
    if (status != VPI_SUCCESS) {
        char error_msg[256];
        vpiGetLastStatusMessage(error_msg, sizeof(error_msg));
        printf("Failed to lock VPI output image: %d (%s)\n", status, error_msg);
        vpiImageDestroy(input_vpi_image);
        gst_buffer_unmap(buffer, &map_info);
        return -1;
    }
    
    // BGRA8 데이터 설정
    int bgra_size = publisher->width * publisher->height * 4;
    *output_size = bgra_size;
    *output_data = (unsigned char*)output_img_data.buffer.pitch.planes[0].data;
    
    printf("VPI GPU processing completed: %dx%d BGRA8 (%d bytes) using NVIDIA workaround\n", 
           publisher->width, publisher->height, bgra_size);
    
    // 정리
    vpiImageUnlock(publisher->vpi_output_image);
    vpiImageDestroy(input_vpi_image);
    gst_buffer_unmap(buffer, &map_info);
    
    return 0;
}
#endif // HAVE_VPI

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
    publisher->current_gst_buffer = NULL;
    publisher->use_thread_processing = 0; // 기본값
    
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
    
    // 인코딩 타입에 따른 설정
    if (encoding_type == GST_ENCODING_V4L2_NVMM_JPEG) {
        publisher->use_thread_processing = 1;
        publisher->publish_compressed = true;
        publisher->use_vpi_gpu_processing = 0;
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_NVMM_JPEG, 
                 camera_id, width, height, width, height);
    } else if (encoding_type == GST_ENCODING_V4L2_VPI_GPU) {
        publisher->use_thread_processing = 1;
        publisher->publish_compressed = false;
        publisher->use_vpi_gpu_processing = 1;
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_VPI_GPU, 
                 camera_id, width, height, width, height);
    } else if (encoding_type == GST_ENCODING_V4L2_BGRx) {
        publisher->use_thread_processing = 0;
        publisher->publish_compressed = false;
        publisher->use_vpi_gpu_processing = 0;
        snprintf(pipeline_str, sizeof(pipeline_str), GST_PIPELINE_TEMPLATE_ECON_V4L2_BGRx, 
                 camera_id, width, height, width, height);
    } else {
        printf("Unsupported encoding type for v4l2src: %s\n", gst_encoding_type_to_string(encoding_type));
        printf("Please use GST_ENCODING_V4L2_NVMM_JPEG, GST_ENCODING_V4L2_VPI_GPU, or GST_ENCODING_V4L2_BGRx\n");
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
    if (encoding_type == GST_ENCODING_V4L2_NVMM_JPEG) {
        snprintf(topic_name, sizeof(topic_name), "/dev/video%d/image_raw/compressed", camera_id);
    } else if (encoding_type == GST_ENCODING_V4L2_VPI_GPU) {
        snprintf(topic_name, sizeof(topic_name), "/dev/video%d/image_raw", camera_id);
    } else if (encoding_type == GST_ENCODING_V4L2_BGRx) {
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
    // iceoryx shared memory 최적화를 위한 QoS 설정
    publisher_options.qos = rmw_qos_profile_sensor_data;
    publisher_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;  // 재전송 비활성화
    publisher_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;       // 메모리 사용량 최소화
    publisher_options.qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;      // liveliness 체크 최소화
 
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
    // camera_info도 동일한 QoS 최적화 적용
    camera_info_options.qos = rmw_qos_profile_sensor_data;
    camera_info_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    camera_info_options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    camera_info_options.qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    
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
    
    // VPI GPU 처리 초기화 (안정성 문제로 임시 비활성화)
    if (publisher->use_vpi_gpu_processing) {
#if HAVE_VPI
        printf("VPI GPU processing initialization temporarily disabled for system stability\n");
        publisher->use_vpi_gpu_processing = 0;  // VPI 비활성화
        
        /*
        // VPI GPU 초기화 코드 - 안정성 문제로 주석 처리
        if (init_vpi_gpu_processing(publisher) != 0) {
            printf("Failed to initialize VPI GPU processing\n");
            rcl_publisher_fini(&publisher->image_publisher, node);
            rcl_publisher_fini(&publisher->camera_info_publisher, node);
            gst_object_unref(publisher->sink);
            gst_object_unref(publisher->pipeline);
            pthread_mutex_destroy(&publisher->mutex);
            free(publisher);
            return NULL;
        }
        */
#else
        printf("Warning: VPI GPU processing requested but VPI not available\n");
        publisher->use_vpi_gpu_processing = 0;  // VPI 비활성화
#endif
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
    
    // 스레드 기반 처리 시작
    if (publisher->use_thread_processing) {
        if (pthread_create(&publisher->processing_thread, NULL, processing_thread_func, publisher) != 0) {
            printf("Failed to create processing thread for camera %d\n", publisher->camera_id);
            publisher->is_running = 0;
            gst_element_set_state(publisher->pipeline, GST_STATE_NULL);
            return -1;
        }
    }
    
    printf("GStreamer ROS Publisher started for camera %d (%s)\n", 
           publisher->camera_id, 
           publisher->use_thread_processing ? "thread mode" : "callback mode");
    return 0;
}

// GStreamer ROS 퍼블리시 정지
int gst_ros_publisher_stop(GstRosPublisher *publisher) {
    if (!publisher || !publisher->is_running) {
        return -1;
    }
    
    publisher->is_running = 0;
    
    // 스레드 기반 처리 종료 대기
    if (publisher->use_thread_processing) {
        pthread_join(publisher->processing_thread, NULL);
    }
    
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
    
    // Zero-copy용 GStreamer 버퍼 레퍼런스 해제
    if (publisher->current_gst_buffer) {
        gst_buffer_unref(publisher->current_gst_buffer);
        publisher->current_gst_buffer = NULL;
    }

    // VPI 3.2 GPU 처리 리소스 해제
    if (publisher->use_vpi_gpu_processing) {
#if HAVE_VPI
        cleanup_vpi_gpu_processing(publisher);
#endif
    }

    // 뮤텍스 해제
    pthread_mutex_destroy(&publisher->mutex);
    
    // 메모리 해제
    free(publisher);
    
    printf("GStreamer ROS Publisher destroyed\n");
} 