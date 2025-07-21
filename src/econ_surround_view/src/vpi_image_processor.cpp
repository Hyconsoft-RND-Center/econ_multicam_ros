#include "econ_surround_view/vpi_image_processor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <chrono>
#include <numeric>
#include <cstring>

namespace econ_surround_view
{

VPIImageProcessor::VPIImageProcessor(int input_width, int input_height, int output_width, int output_height)
  : input_width_(input_width)
  , input_height_(input_height)
  , output_width_(output_width)
  , output_height_(output_height)
  , initialized_(false)
  , stream_(nullptr)
  , avg_processing_time_ms_(0.0)
  , frame_count_(0)
{
  // VPI 이미지들 초기화
  for (int i = 0; i < 4; ++i) {
    input_vpi_[i] = nullptr;
    output_vpi_[i] = nullptr;
    temp_vpi_[i] = nullptr;
    camera_configs_[i].intrinsics_set = false;
    camera_configs_[i].extrinsics_set = false;
  }

  // VPI 리소스 초기화
  initializeVPIImages();
}

VPIImageProcessor::~VPIImageProcessor()
{
  cleanupVPIResources();
}

void VPIImageProcessor::initializeVPIImages()
{
  try {
    // VPI 스트림 생성 - CUDA와 CPU 백엔드 모두 사용
    CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &stream_), "VPI 스트림 생성");

    // 4개 카메라용 VPI 이미지들 생성 - CUDA와 CPU 백엔드 모두 사용
    // CPU 백엔드가 필요하여 이미지를 호스트 메모리에 매핑할 수 있도록 함
    uint64_t backend_flags = VPI_BACKEND_CUDA | VPI_BACKEND_CPU;
    
    for (int i = 0; i < 4; ++i) {
      // 입력 이미지 (BGR8 형식)
      CHECK_STATUS(vpiImageCreate(input_width_, input_height_, VPI_IMAGE_FORMAT_BGR8, 
                                 backend_flags, &input_vpi_[i]), 
                  "입력 VPI 이미지 생성");

      // 출력 이미지 (크기 조정된 BGR8 형식)
      CHECK_STATUS(vpiImageCreate(output_width_, output_height_, VPI_IMAGE_FORMAT_BGR8, 
                                 backend_flags, &output_vpi_[i]), 
                  "출력 VPI 이미지 생성");

      // 임시 이미지 (중간 처리용)
      CHECK_STATUS(vpiImageCreate(input_width_, input_height_, VPI_IMAGE_FORMAT_BGR8, 
                                 backend_flags, &temp_vpi_[i]), 
                  "임시 VPI 이미지 생성");
    }

    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("VPIImageProcessor"), 
                "VPI 이미지 프로세서 초기화 완료 (%dx%d -> %dx%d)", 
                input_width_, input_height_, output_width_, output_height_);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "VPI 초기화 실패: %s", e.what());
    cleanupVPIResources();
    throw;
  }
}

void VPIImageProcessor::cleanupVPIResources()
{
  // VPI 이미지들 해제
  for (int i = 0; i < 4; ++i) {
    if (input_vpi_[i] != nullptr) {
      vpiImageDestroy(input_vpi_[i]);
      input_vpi_[i] = nullptr;
    }
    if (output_vpi_[i] != nullptr) {
      vpiImageDestroy(output_vpi_[i]);
      output_vpi_[i] = nullptr;
    }
    if (temp_vpi_[i] != nullptr) {
      vpiImageDestroy(temp_vpi_[i]);
      temp_vpi_[i] = nullptr;
    }
  }

  // 스트림 해제
  if (stream_ != nullptr) {
    vpiStreamDestroy(stream_);
    stream_ = nullptr;
  }

  initialized_ = false;
  RCLCPP_INFO(rclcpp::get_logger("VPIImageProcessor"), "VPI 리소스 정리 완료");
}

bool VPIImageProcessor::set_camera_intrinsics(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, int camera_id)
{
  if (camera_id < 0 || camera_id >= 4) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), "잘못된 카메라 ID: %d", camera_id);
    return false;
  }

  try {
    camera_configs_[camera_id].camera_matrix = camera_matrix.clone();
    camera_configs_[camera_id].dist_coeffs = dist_coeffs.clone();
    camera_configs_[camera_id].intrinsics_set = true;

    // 카메라 파라미터 로깅
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);

    RCLCPP_INFO(rclcpp::get_logger("VPIImageProcessor"), 
                "카메라 %d 내부 파라미터 설정: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                camera_id, fx, fy, cx, cy);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 내부 파라미터 설정 실패: %s", camera_id, e.what());
    return false;
  }
}

bool VPIImageProcessor::set_camera_extrinsics(const cv::Mat& homography, int camera_id)
{
  if (camera_id < 0 || camera_id >= 4) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), "잘못된 카메라 ID: %d", camera_id);
    return false;
  }

  try {
    camera_configs_[camera_id].homography = homography.clone();
    camera_configs_[camera_id].extrinsics_set = true;

    RCLCPP_INFO(rclcpp::get_logger("VPIImageProcessor"), 
                "카메라 %d 외부 파라미터 설정 완료", camera_id);
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 외부 파라미터 설정 실패: %s", camera_id, e.what());
    return false;
  }
}

std::vector<cv::Mat> VPIImageProcessor::process_camera_images(const std::vector<cv::Mat>& input_images)
{
  if (!initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), "VPI가 초기화되지 않았습니다.");
    return {};
  }

  if (input_images.size() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "입력 이미지 개수 오류: 4개 필요, %zu개 제공", input_images.size());
    return {};
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<cv::Mat> output_images(4);

  try {
    // 각 카메라 이미지 처리
    for (int i = 0; i < 4; ++i) {
      if (input_images[i].empty()) {
        RCLCPP_WARN(rclcpp::get_logger("VPIImageProcessor"), 
                    "카메라 %d 이미지가 비어있습니다.", i);
        output_images[i] = cv::Mat::zeros(output_height_, output_width_, CV_8UC3);
        continue;
      }

      // 1. OpenCV 이미지를 VPI로 업로드
      uploadToVPI(input_images[i], input_vpi_[i]);

      // 2. 렌즈 왜곡 보정 (내부 파라미터가 설정된 경우)
      VPIImage processed_img = input_vpi_[i];
      if (camera_configs_[i].intrinsics_set) {
        processed_img = applyLensDistortionCorrection(processed_img, i);
      }

      // 3. 투시 변환 (외부 파라미터가 설정된 경우)
      if (camera_configs_[i].extrinsics_set) {
        processed_img = applyPerspectiveTransform(processed_img, i);
      }

      // 4. 크기 조정 (기본 처리)
      applyResize(processed_img, output_vpi_[i]);

      // 5. VPI에서 OpenCV로 다운로드
      downloadFromVPI(output_vpi_[i], output_images[i]);
    }

    // 스트림 동기화
    CHECK_STATUS(vpiStreamSync(stream_), "스트림 동기화");

    // 성능 측정
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double processing_time_ms = duration.count() / 1000.0;

    processing_times_.push_back(processing_time_ms);
    if (processing_times_.size() > 100) {
      processing_times_.erase(processing_times_.begin());
    }

    double sum = std::accumulate(processing_times_.begin(), processing_times_.end(), 0.0);
    avg_processing_time_ms_ = sum / processing_times_.size();
    frame_count_++;

    RCLCPP_DEBUG(rclcpp::get_logger("VPIImageProcessor"), 
                 "프레임 %zu 처리 완료: %.2f ms (평균: %.2f ms)", 
                 frame_count_, processing_time_ms, avg_processing_time_ms_);

    return output_images;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "이미지 처리 중 오류: %s", e.what());
    return {};
  }
}

void VPIImageProcessor::uploadToVPI(const cv::Mat& cv_image, VPIImage vpi_image)
{
  // 입력 이미지 검증
  if (cv_image.empty() || cv_image.cols <= 0 || cv_image.rows <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), "VPI 업로드용 입력 이미지가 유효하지 않습니다");
    return;
  }
  
  VPIImageData img_data;
  CHECK_STATUS(vpiImageLockData(vpi_image, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &img_data), 
               "VPI 이미지 데이터 락");
  
  VPIImageBufferPitchLinear& host_data = img_data.buffer.pitch;
  
  // BGR8 형식에 대한 올바른 평면 수 확인
  if (host_data.numPlanes < 1) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), "VPI 이미지의 평면 수가 유효하지 않습니다");
    vpiImageUnlock(vpi_image);
    return;
  }
  
  // OpenCV에서 VPI로 데이터 복사
  for (int y = 0; y < cv_image.rows; ++y) {
    const uint8_t* src_row = cv_image.ptr<uint8_t>(y);
    uint8_t* dst_row = reinterpret_cast<uint8_t*>(host_data.planes[0].data) + y * host_data.planes[0].pitchBytes;
    std::memcpy(dst_row, src_row, cv_image.cols * cv_image.channels());
  }
  
  CHECK_STATUS(vpiImageUnlock(vpi_image), "VPI 이미지 언락");
}

void VPIImageProcessor::downloadFromVPI(VPIImage vpi_image, cv::Mat& cv_image)
{
  static bool first_call = true;
  
  // VPI 이미지 정보 먼저 가져오기
  int32_t vpi_width, vpi_height;
  VPIImageFormat vpi_format;
  CHECK_STATUS(vpiImageGetSize(vpi_image, &vpi_width, &vpi_height), "VPI 이미지 크기 조회");
  CHECK_STATUS(vpiImageGetFormat(vpi_image, &vpi_format), "VPI 이미지 형식 조회");
  
  if (first_call) {
    RCLCPP_INFO(rclcpp::get_logger("VPIImageProcessor"), 
                "VPI 이미지 정보: %dx%d, 형식: %ld", vpi_width, vpi_height, static_cast<long>(vpi_format));
    first_call = false;
  }
  
  VPIImageData img_data;
  CHECK_STATUS(vpiImageLockData(vpi_image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &img_data), 
               "VPI 이미지 데이터 락");
  
  VPIImageBufferPitchLinear& host_data = img_data.buffer.pitch;
  
  // 실제 VPI 이미지 크기 사용
  if (vpi_width <= 0 || vpi_height <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "유효하지 않은 VPI 이미지 크기: %dx%d", vpi_width, vpi_height);
    vpiImageUnlock(vpi_image);
    return;
  }
  
  cv_image.create(vpi_height, vpi_width, CV_8UC3);
  
  // 실제 크기를 사용하여 VPI에서 OpenCV로 데이터 복사
  for (int y = 0; y < vpi_height; ++y) {
    const uint8_t* src_row = reinterpret_cast<const uint8_t*>(host_data.planes[0].data) + y * host_data.planes[0].pitchBytes;
    uint8_t* dst_row = cv_image.ptr<uint8_t>(y);
    std::memcpy(dst_row, src_row, vpi_width * 3);
  }
  
  CHECK_STATUS(vpiImageUnlock(vpi_image), "VPI 이미지 언락");
}

VPIImage VPIImageProcessor::applyLensDistortionCorrection(VPIImage input, int camera_id)
{
  if (!camera_configs_[camera_id].intrinsics_set) {
    RCLCPP_DEBUG(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 내부 파라미터가 설정되지 않음", camera_id);
    return input;
  }

  try {
    const cv::Mat& camera_matrix = camera_configs_[camera_id].camera_matrix;
    const cv::Mat& dist_coeffs = camera_configs_[camera_id].dist_coeffs;

    // VPI 카메라 내부 파라미터 설정
    VPICameraIntrinsic K;
    K[0][0] = camera_matrix.at<double>(0, 0); // fx
    K[0][1] = 0.0;
    K[0][2] = camera_matrix.at<double>(0, 2); // cx
    K[1][0] = 0.0;
    K[1][1] = camera_matrix.at<double>(1, 1); // fy
    K[1][2] = camera_matrix.at<double>(1, 2); // cy

    // VPI 외부 파라미터 (단위 행렬 - 왜곡 보정만)
    VPICameraExtrinsic X = {};
    X[0][0] = X[1][1] = X[2][2] = 1.0;

    // Polynomial 왜곡 모델 설정
    VPIPolynomialLensDistortionModel poly_model = {};
    if (dist_coeffs.rows >= 4) {
      poly_model.k1 = dist_coeffs.at<double>(0, 0);
      poly_model.k2 = dist_coeffs.at<double>(1, 0);
      poly_model.p1 = dist_coeffs.at<double>(2, 0);
      poly_model.p2 = dist_coeffs.at<double>(3, 0);
      poly_model.k3 = (dist_coeffs.rows >= 5) ? dist_coeffs.at<double>(4, 0) : 0.0;
      poly_model.k4 = poly_model.k5 = poly_model.k6 = 0.0;
    }

    // Warp Map 생성
    VPIWarpMap warp_map = {};
    warp_map.grid.numHorizRegions = 1;
    warp_map.grid.numVertRegions = 1;
    warp_map.grid.regionWidth[0] = input_width_;
    warp_map.grid.regionHeight[0] = input_height_;
    warp_map.grid.horizInterval[0] = 1;
    warp_map.grid.vertInterval[0] = 1;

    CHECK_STATUS(vpiWarpMapAllocData(&warp_map), "Warp Map 할당");

    // 렌즈 왜곡 보정 맵 생성
    CHECK_STATUS(vpiWarpMapGenerateFromPolynomialLensDistortionModel(
      K, X, K, &poly_model, &warp_map), "렌즈 왜곡 보정 맵 생성");

    // Remap payload 생성 (CUDA와 CPU 백엔드 모두 사용)
    VPIPayload remap_payload;
    CHECK_STATUS(vpiCreateRemap(VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &warp_map, &remap_payload), 
                 "Remap payload 생성");

    // 왜곡 보정 실행 (CUDA 백엔드 사용 - GPU 가속)
    CHECK_STATUS(vpiSubmitRemap(stream_, VPI_BACKEND_CUDA, remap_payload, 
                               input, temp_vpi_[camera_id], 
                               VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0), 
                 "렌즈 왜곡 보정 실행");

    // 리소스 정리
    vpiPayloadDestroy(remap_payload);
    vpiWarpMapFreeData(&warp_map);

    RCLCPP_DEBUG(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 렌즈 왜곡 보정 완료", camera_id);

    return temp_vpi_[camera_id];

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 렌즈 왜곡 보정 실패: %s", camera_id, e.what());
    return input;
  }
}

VPIImage VPIImageProcessor::applyPerspectiveTransform(VPIImage input, int camera_id)
{
  if (!camera_configs_[camera_id].extrinsics_set) {
    RCLCPP_DEBUG(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 외부 파라미터가 설정되지 않음", camera_id);
    return input;
  }

  try {
    const cv::Mat& homography = camera_configs_[camera_id].homography;

    // OpenCV 호모그래피를 VPI 투시 변환으로 변환
    VPIPerspectiveTransform transform;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        transform[i][j] = homography.at<double>(i, j);
      }
    }

    // 출력 이미지 생성 (투시 변환된 크기) - CPU 백엔드 포함
    VPIImage transformed_output;
    CHECK_STATUS(vpiImageCreate(output_width_, output_height_, VPI_IMAGE_FORMAT_BGR8, 
                               VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &transformed_output), 
                 "투시 변환 출력 이미지 생성");

    // 투시 변환 실행 (CUDA 백엔드 사용 - GPU 가속)
    CHECK_STATUS(vpiSubmitPerspectiveWarp(stream_, VPI_BACKEND_CUDA, 
                                         input, transform, transformed_output, 
                                         nullptr, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0), 
                 "투시 변환 실행");

    RCLCPP_DEBUG(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 투시 변환 완료", camera_id);

    // 이전 temp_vpi_ 이미지 교체
    std::swap(temp_vpi_[camera_id], transformed_output);
    vpiImageDestroy(transformed_output);

    return temp_vpi_[camera_id];

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), 
                 "카메라 %d 투시 변환 실패: %s", camera_id, e.what());
    return input;
  }
}

void VPIImageProcessor::applyResize(VPIImage input, VPIImage output)
{
  // VPI Rescale 알고리즘 사용 (CUDA 백엔드 사용 - GPU 가속)
  CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, input, output, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0), 
               "이미지 크기 조정");
}

void VPIImageProcessor::CHECK_STATUS(VPIStatus status, const std::string& operation)
{
  if (status != VPI_SUCCESS) {
    char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];
    vpiGetLastStatusMessage(buffer, sizeof(buffer));
    
    std::string error_msg = operation + " 실패: " + vpiStatusGetName(status) + " - " + buffer;
    RCLCPP_ERROR(rclcpp::get_logger("VPIImageProcessor"), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }
}

}  // namespace econ_surround_view 