#ifndef ECON_SURROUND_VIEW__CAMERA_CALIBRATION_MANAGER_HPP_
#define ECON_SURROUND_VIEW__CAMERA_CALIBRATION_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace econ_surround_view
{

/**
 * @brief 카메라별 캘리브레이션 데이터 구조체
 */
struct CameraCalibrationData
{
  // 내부 파라미터
  cv::Mat camera_matrix;           // 3x3 카메라 내부 행렬 K
  cv::Mat distortion_coefficients; // 왜곡 계수 (k1, k2, k3, k4) 또는 (k1, k2, p1, p2, k3)
  cv::Size image_size;             // 이미지 크기
  
  // 외부 파라미터 (카메라 -> 차량 좌표계)
  cv::Mat rotation_matrix;         // 3x3 회전 행렬 R
  cv::Mat translation_vector;      // 3x1 평행이동 벡터 t
  cv::Mat homography_matrix;       // 3x3 호모그래피 행렬 (평면 투영용)
  
  // 추가 정보
  std::string camera_name;         // 카메라 이름 (front, back, left, right)
  bool is_fisheye;                // 어안 렌즈 여부
  bool is_calibrated;             // 캘리브레이션 완료 여부
  double reprojection_error;      // 재투영 오차 (RMS)
  
  // 탑뷰 변환 영역 설정
  cv::Rect roi_in_topview;        // 탑뷰에서 이 카메라가 담당할 영역
  cv::Point2f ground_center;      // 지면 중심점 (mm 단위)
  double pixels_per_meter;        // 픽셀/미터 비율
};

/**
 * @brief 카메라 캘리브레이션 매니저 클래스
 * 
 * 4개 카메라의 캘리브레이션 데이터를 로드, 저장, 관리하는 클래스
 * - 내부/외부 파라미터 로드
 * - 체스보드를 이용한 자동 캘리브레이션
 * - 호모그래피 계산 및 검증
 * - 캘리브레이션 품질 평가
 */
class CameraCalibrationManager
{
public:
  /**
   * @brief 생성자
   * @param node ROS2 노드 포인터 (로깅용)
   */
  explicit CameraCalibrationManager(rclcpp::Node* node);

  /**
   * @brief 소멸자
   */
  ~CameraCalibrationManager() = default;

  /**
   * @brief 설정 파일에서 캘리브레이션 데이터 로드
   * @param config_file_path YAML 설정 파일 경로
   * @return 로드 성공 여부
   */
  bool load_calibration_from_file(const std::string& config_file_path);

  /**
   * @brief 개별 카메라 캘리브레이션 데이터 저장
   * @param camera_id 카메라 ID (0:front, 1:back, 2:left, 3:right)
   * @param calib_data 캘리브레이션 데이터
   * @return 저장 성공 여부
   */
  bool set_camera_calibration(int camera_id, const CameraCalibrationData& calib_data);

  /**
   * @brief 카메라 캘리브레이션 데이터 조회
   * @param camera_id 카메라 ID
   * @return 캘리브레이션 데이터 (nullptr: 실패)
   */
  const CameraCalibrationData* get_camera_calibration(int camera_id) const;

  /**
   * @brief 체스보드를 이용한 자동 캘리브레이션 실행
   * @param camera_id 카메라 ID
   * @param calibration_images 체스보드 이미지들
   * @param board_size 체스보드 크기 (가로x세로 코너 수)
   * @param square_size 체스보드 정사각형 크기 (mm)
   * @return 캘리브레이션 성공 여부
   */
  bool calibrate_camera_from_chessboard(
    int camera_id,
    const std::vector<cv::Mat>& calibration_images,
    const cv::Size& board_size,
    float square_size);

  /**
   * @brief 지면 평면 기반 호모그래피 계산
   * @param camera_id 카메라 ID
   * @param ground_points 지면의 4개 기준점 (실제 좌표, mm)
   * @param image_points 이미지에서의 해당 4개 점 (픽셀 좌표)
   * @return 계산 성공 여부
   */
  bool calculate_ground_homography(
    int camera_id,
    const std::vector<cv::Point2f>& ground_points,
    const std::vector<cv::Point2f>& image_points);

  /**
   * @brief 캘리브레이션 품질 검증
   * @param camera_id 카메라 ID
   * @param test_images 테스트 이미지들
   * @return 평균 재투영 오차 (픽셀)
   */
  double validate_calibration_quality(int camera_id, const std::vector<cv::Mat>& test_images);

  /**
   * @brief 모든 카메라 캘리브레이션 완료 여부 확인
   * @return 모든 카메라 캘리브레이션 완료 여부
   */
  bool are_all_cameras_calibrated() const;

  /**
   * @brief 캘리브레이션 데이터를 파일로 저장
   * @param output_file_path 출력 파일 경로
   * @return 저장 성공 여부
   */
  bool save_calibration_to_file(const std::string& output_file_path) const;

  /**
   * @brief 캘리브레이션 상태 요약 출력
   */
  void print_calibration_summary() const;

  /**
   * @brief 이미지 언디스토트 (왜곡 보정)
   * @param camera_id 카메라 ID
   * @param distorted_image 왜곡된 입력 이미지
   * @return 왜곡 보정된 이미지
   */
  cv::Mat undistort_image(int camera_id, const cv::Mat& distorted_image) const;

  /**
   * @brief 이미지를 탑뷰로 변환
   * @param camera_id 카메라 ID  
   * @param input_image 입력 이미지
   * @param output_size 출력 탑뷰 이미지 크기
   * @return 탑뷰 변환된 이미지
   */
  cv::Mat transform_to_topview(
    int camera_id, 
    const cv::Mat& input_image, 
    const cv::Size& output_size) const;

  /**
   * @brief 기본 캘리브레이션 파라미터 생성 (개발/테스트용)
   * @param image_width 이미지 너비
   * @param image_height 이미지 높이
   * @return 생성 성공 여부
   */
  bool create_default_calibration(int image_width, int image_height);

private:
  // 내부 유틸리티 함수들
  bool validate_calibration_data(const CameraCalibrationData& data) const;
  cv::Mat create_camera_matrix(double fx, double fy, double cx, double cy) const;
  cv::Mat create_distortion_coefficients(double k1, double k2, double k3, double k4) const;
  cv::Mat create_default_homography(int camera_id, int image_width, int image_height) const;
  cv::Rect create_default_roi(int camera_id, int output_width, int output_height) const;
  
  // YAML 파일 I/O 함수들
  bool load_camera_data_from_yaml(const std::string& file_path, int camera_id);
  bool save_camera_data_to_yaml(const std::string& file_path, int camera_id) const;

  // 어안 렌즈 전용 캘리브레이션 함수들
  bool calibrate_fisheye_camera(
    int camera_id,
    const std::vector<cv::Mat>& calibration_images,
    const cv::Size& board_size,
    float square_size);

  // 멤버 변수들
  rclcpp::Node* node_;  // ROS2 노드 (로깅용)
  std::map<int, CameraCalibrationData> calibrations_;  // 카메라별 캘리브레이션 데이터
  
  // 카메라 이름 매핑
  static const std::map<int, std::string> CAMERA_NAMES;
  
  // 기본 설정값들
  static constexpr int NUM_CAMERAS = 4;
  static constexpr double DEFAULT_FOCAL_LENGTH = 800.0;
  static constexpr double DEFAULT_PIXEL_SIZE = 0.0055;  // mm
};

}  // namespace econ_surround_view

#endif  // ECON_SURROUND_VIEW__CAMERA_CALIBRATION_MANAGER_HPP_ 