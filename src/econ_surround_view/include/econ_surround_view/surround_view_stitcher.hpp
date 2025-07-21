#ifndef ECON_SURROUND_VIEW__SURROUND_VIEW_STITCHER_HPP_
#define ECON_SURROUND_VIEW__SURROUND_VIEW_STITCHER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace econ_surround_view
{

/**
 * @brief 블렌딩 방법 열거형
 */
enum class BlendingMethod
{
  SIMPLE_ALPHA,      // 단순 알파 블렌딩
  WEIGHTED_AVERAGE,  // 가중 평균 블렌딩
  MULTIBAND,         // 멀티밴드 블렌딩
  FEATHERING,        // 페더링 블렌딩
  SEAM_CUTTING       // 심 커팅 블렌딩
};

/**
 * @brief 카메라별 ROI 및 블렌딩 영역 정보
 */
struct CameraRegion
{
  cv::Rect roi;                    // 전체 서라운드 뷰에서 이 카메라가 담당하는 영역
  cv::Rect valid_area;             // 실제 유효한 데이터가 있는 영역
  cv::Mat blend_mask;              // 블렌딩용 마스크 (0~255)
  cv::Mat weight_mask;             // 가중치 마스크 (0.0~1.0)
  std::vector<cv::Point> contour;  // 영역 경계선
  bool is_active;                  // 활성화 여부
};

/**
 * @brief 서라운드 뷰 스티처 클래스
 * 
 * 4개 카메라의 탑뷰 이미지를 하나의 360도 서라운드 뷰로 합성하는 클래스
 * - 각 카메라 영역 설정 및 관리
 * - 겹치는 영역의 블렌딩 처리
 * - 차량 모델 오버레이
 * - 실시간 성능 최적화
 */
class SurroundViewStitcher
{
public:
  /**
   * @brief 생성자
   * @param output_width 출력 서라운드 뷰 이미지 너비
   * @param output_height 출력 서라운드 뷰 이미지 높이
   * @param node ROS2 노드 포인터 (로깅용)
   */
  SurroundViewStitcher(int output_width, int output_height, rclcpp::Node* node);

  /**
   * @brief 소멸자
   */
  ~SurroundViewStitcher() = default;

  /**
   * @brief 카메라별 ROI 영역 설정
   * @param camera_id 카메라 ID (0:front, 1:back, 2:left, 3:right)
   * @param roi 해당 카메라가 담당할 서라운드 뷰 영역
   * @param blend_width 블렌딩 영역 너비 (픽셀)
   * @return 설정 성공 여부
   */
  bool set_camera_region(int camera_id, const cv::Rect& roi, int blend_width = 50);

  /**
   * @brief 블렌딩 방법 설정
   * @param method 블렌딩 방법
   * @param alpha 알파 블렌딩 계수 (0.0~1.0)
   * @return 설정 성공 여부
   */
  bool set_blending_method(BlendingMethod method, double alpha = 0.5);

  /**
   * @brief 4개 카메라 이미지를 서라운드 뷰로 합성
   * @param camera_images 4개 카메라의 탑뷰 이미지 [front, back, left, right]
   * @return 합성된 서라운드 뷰 이미지
   */
  cv::Mat stitch_images(const std::vector<cv::Mat>& camera_images);

  /**
   * @brief 차량 모델 오버레이 설정
   * @param vehicle_model_path 차량 모델 이미지 파일 경로
   * @param vehicle_center 차량 중심 위치 (서라운드 뷰 좌표)
   * @param vehicle_size 차량 크기 (픽셀)
   * @return 설정 성공 여부
   */
  bool set_vehicle_overlay(
    const std::string& vehicle_model_path,
    const cv::Point2f& vehicle_center,
    const cv::Size& vehicle_size);

  /**
   * @brief 그리드 오버레이 설정
   * @param enable_grid 그리드 표시 여부
   * @param grid_spacing 그리드 간격 (픽셀)
   * @param grid_color 그리드 색상
   * @return 설정 성공 여부
   */
  bool set_grid_overlay(bool enable_grid, int grid_spacing = 50, const cv::Scalar& grid_color = cv::Scalar(128, 128, 128));

  /**
   * @brief 카메라별 활성화/비활성화 설정
   * @param camera_id 카메라 ID
   * @param active 활성화 여부
   */
  void set_camera_active(int camera_id, bool active);

  /**
   * @brief 동적 ROI 조정 (런타임에 영역 변경)
   * @param camera_id 카메라 ID
   * @param new_roi 새로운 ROI 영역
   * @return 조정 성공 여부
   */
  bool adjust_camera_region(int camera_id, const cv::Rect& new_roi);

  /**
   * @brief 색상 보정 설정
   * @param enable_color_correction 색상 보정 활성화 여부
   * @param reference_camera_id 기준 카메라 ID (색상 기준)
   * @return 설정 성공 여부
   */
  bool set_color_correction(bool enable_color_correction, int reference_camera_id = 0);

  /**
   * @brief 스티칭 성능 통계 조회
   * @return 평균 처리 시간 (ms)
   */
  double get_average_stitching_time() const { return avg_stitching_time_ms_; }

  /**
   * @brief 현재 설정 요약 출력
   */
  void print_configuration_summary() const;

  /**
   * @brief 기본 레이아웃 설정 (개발/테스트용)
   * @param layout_type 레이아웃 타입 ("cross", "square", "custom")
   * @return 설정 성공 여부
   */
  bool setup_default_layout(const std::string& layout_type = "cross");

private:
  // 초기화 및 설정 함수들
  void initialize_blend_masks();
  void create_feathering_mask(int camera_id, int blend_width);
  void create_weight_masks();

  // 블렌딩 알고리즘들
  cv::Mat alpha_blend(const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& masks);
  cv::Mat weighted_average_blend(const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& weights);
  cv::Mat multiband_blend(const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& masks);
  cv::Mat feathering_blend(const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& masks);

  // 색상 보정 함수들
  void calculate_color_correction_parameters();
  cv::Mat apply_color_correction(const cv::Mat& image, int camera_id);

  // 오버레이 함수들
  void draw_vehicle_overlay(cv::Mat& output_image);
  void draw_grid_overlay(cv::Mat& output_image);
  void draw_camera_labels(cv::Mat& output_image);

  // 유틸리티 함수들
  bool validate_camera_images(const std::vector<cv::Mat>& images);
  cv::Rect calculate_overlap_region(const cv::Rect& roi1, const cv::Rect& roi2);
  cv::Mat create_distance_transform_mask(const cv::Size& size, const cv::Rect& roi);
  void place_image_in_roi(cv::Mat& target, const cv::Mat& source, const cv::Rect& roi, int camera_id);
  void apply_blending(cv::Mat& surround_view, const std::vector<cv::Mat>& camera_images);

  // 멤버 변수들
  int output_width_, output_height_;
  rclcpp::Node* node_;

  // 카메라 영역 및 블렌딩 설정
  std::array<CameraRegion, 4> camera_regions_;
  BlendingMethod blending_method_;
  double alpha_value_;
  int blend_width_;

  // 색상 보정 관련
  bool enable_color_correction_;
  int reference_camera_id_;
  std::array<cv::Mat, 4> color_correction_matrices_;

  // 오버레이 설정
  cv::Mat vehicle_model_;
  cv::Point2f vehicle_center_;
  cv::Size vehicle_size_;
  bool enable_vehicle_overlay_;
  
  bool enable_grid_overlay_;
  int grid_spacing_;
  cv::Scalar grid_color_;

  // 성능 모니터링
  mutable std::vector<double> stitching_times_;
  mutable double avg_stitching_time_ms_;
  mutable size_t frame_count_;

  // 상수 정의
  static constexpr int NUM_CAMERAS = 4;
  static constexpr double DEFAULT_ALPHA = 0.5;
  static constexpr int DEFAULT_BLEND_WIDTH = 50;
  
  // 카메라 이름 매핑
  static const std::array<std::string, 4> CAMERA_NAMES;
};

}  // namespace econ_surround_view

#endif  // ECON_SURROUND_VIEW__SURROUND_VIEW_STITCHER_HPP_ 