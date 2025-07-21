#ifndef ECON_SURROUND_VIEW__VPI_IMAGE_PROCESSOR_HPP_
#define ECON_SURROUND_VIEW__VPI_IMAGE_PROCESSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/Remap.h>
#include <vpi/algo/PerspectiveWarp.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/OpenCVInterop.hpp>

namespace econ_surround_view
{

/**
 * @brief VPI 3.2를 사용한 이미지 처리 클래스
 * 
 * 실제 차량 카메라 배치:
 * - Cam1 (전방): (0.575, 0, 0.0945) - 0도
 * - Cam2 (우측): (0, 0.303, 0.1815) - 90도  
 * - Cam3 (좌측): (0, -0.303, 0.1815) - -90도
 * - Cam4 (후방): (-0.403, 0, 0.1295) - 180도
 */
class VPIImageProcessor
{
public:
  /**
   * @brief 생성자
   * @param input_width 입력 이미지 너비 (1920)
   * @param input_height 입력 이미지 높이 (1080)
   * @param output_width 출력 이미지 너비 (512)
   * @param output_height 출력 이미지 높이 (512)
   */
  VPIImageProcessor(int input_width, int input_height, int output_width, int output_height);
  
  /**
   * @brief 소멸자
   */
  ~VPIImageProcessor();

  /**
   * @brief 카메라 내부 파라미터 설정
   * @param camera_matrix 3x3 카메라 행렬
   * @param dist_coeffs 왜곡 계수
   * @param camera_id 카메라 ID (0:전방, 1:후방, 2:좌측, 3:우측)
   * @return 성공 여부
   */
  bool set_camera_intrinsics(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, int camera_id);

  /**
   * @brief 카메라 외부 파라미터 설정 (호모그래피)
   * @param homography 3x3 호모그래피 행렬
   * @param camera_id 카메라 ID
   * @return 성공 여부
   */
  bool set_camera_extrinsics(const cv::Mat& homography, int camera_id);

  /**
   * @brief 4개 카메라 이미지 처리 (간단한 크기 조정 및 변환)
   * @param input_images 4개 카메라 이미지 [전방, 후방, 좌측, 우측]
   * @return 처리된 4개 이미지 (크기 조정됨)
   */
  std::vector<cv::Mat> process_camera_images(const std::vector<cv::Mat>& input_images);

  /**
   * @brief VPI 초기화 상태 확인
   * @return 초기화 완료 여부
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief 처리 통계 정보 반환
   * @return 평균 처리 시간 (ms)
   */
  double get_average_processing_time() const { return avg_processing_time_ms_; }

private:
  // 초기화 및 정리 함수들
  void initializeVPIImages();
  void cleanupVPIResources();
  
  // VPI 유틸리티 함수들
  void uploadToVPI(const cv::Mat& cv_image, VPIImage vpi_image);
  void downloadFromVPI(VPIImage vpi_image, cv::Mat& cv_image);
  void CHECK_STATUS(VPIStatus status, const std::string& operation = "");

  // VPI 이미지 처리 함수들
  VPIImage applyLensDistortionCorrection(VPIImage input, int camera_id);
  VPIImage applyPerspectiveTransform(VPIImage input, int camera_id);
  void applyResize(VPIImage input, VPIImage output);

  // 멤버 변수들
  int input_width_, input_height_;
  int output_width_, output_height_;
  bool initialized_;

  // VPI 리소스들
  VPIStream stream_;
  VPIImage input_vpi_[4];     // 4개 카메라 입력 이미지
  VPIImage output_vpi_[4];    // 4개 카메라 출력 이미지
  VPIImage temp_vpi_[4];      // 임시 이미지들

  // 성능 모니터링
  mutable std::vector<double> processing_times_;
  mutable double avg_processing_time_ms_;
  mutable size_t frame_count_;

  // 카메라 설정
  struct CameraConfig {
    bool intrinsics_set;
    bool extrinsics_set;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat homography;
  };
  CameraConfig camera_configs_[4];
};

}  // namespace econ_surround_view

#endif  // ECON_SURROUND_VIEW__VPI_IMAGE_PROCESSOR_HPP_ 