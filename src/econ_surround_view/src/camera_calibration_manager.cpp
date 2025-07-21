#include "econ_surround_view/camera_calibration_manager.hpp"

#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>

namespace econ_surround_view
{

// 카메라 이름 매핑 정의
const std::map<int, std::string> CameraCalibrationManager::CAMERA_NAMES = {
  {0, "front"},
  {1, "back"},
  {2, "left"},
  {3, "right"}
};

CameraCalibrationManager::CameraCalibrationManager(rclcpp::Node* node)
: node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "카메라 캘리브레이션 매니저 초기화");
}

bool CameraCalibrationManager::load_calibration_from_file(const std::string& config_file_path)
{
  RCLCPP_INFO(node_->get_logger(), "캘리브레이션 파일 로드 시도: %s", config_file_path.c_str());
  
  // TODO: YAML 파일에서 캘리브레이션 데이터 로드 구현
  // 현재는 간단한 구현만 제공
  
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "캘리브레이션 파일을 열 수 없음: %s", config_file_path.c_str());
    return false;
  }

  // TODO: 실제 YAML 파싱 구현
  file.close();
  
  RCLCPP_WARN(node_->get_logger(), "YAML 파싱 미구현. 기본 캘리브레이션 사용");
  return false;  // 현재는 항상 실패로 처리하여 기본값 사용
}

bool CameraCalibrationManager::set_camera_calibration(int camera_id, const CameraCalibrationData& calib_data)
{
  if (camera_id < 0 || camera_id >= NUM_CAMERAS) {
    RCLCPP_ERROR(node_->get_logger(), "잘못된 카메라 ID: %d", camera_id);
    return false;
  }

  if (!validate_calibration_data(calib_data)) {
    RCLCPP_ERROR(node_->get_logger(), "유효하지 않은 캘리브레이션 데이터");
    return false;
  }

  calibrations_[camera_id] = calib_data;
  calibrations_[camera_id].is_calibrated = true;

  RCLCPP_INFO(node_->get_logger(), "카메라 %d (%s) 캘리브레이션 설정 완료", 
              camera_id, CAMERA_NAMES.at(camera_id).c_str());
  
  return true;
}

const CameraCalibrationData* CameraCalibrationManager::get_camera_calibration(int camera_id) const
{
  auto it = calibrations_.find(camera_id);
  if (it != calibrations_.end() && it->second.is_calibrated) {
    return &it->second;
  }
  return nullptr;
}

bool CameraCalibrationManager::create_default_calibration(int image_width, int image_height)
{
  RCLCPP_INFO(node_->get_logger(), "기본 캘리브레이션 파라미터 생성 (%dx%d)", image_width, image_height);

  for (int camera_id = 0; camera_id < NUM_CAMERAS; ++camera_id) {
    CameraCalibrationData calib_data;
    
    // 기본 카메라 내부 파라미터 생성
    double fx = DEFAULT_FOCAL_LENGTH;
    double fy = DEFAULT_FOCAL_LENGTH;
    double cx = image_width / 2.0;
    double cy = image_height / 2.0;
    
    calib_data.camera_matrix = create_camera_matrix(fx, fy, cx, cy);
    calib_data.distortion_coefficients = create_distortion_coefficients(0.0, 0.0, 0.0, 0.0);  // 왜곡 없음
    calib_data.image_size = cv::Size(image_width, image_height);
    
    // 기본 외부 파라미터 (단위 행렬)
    calib_data.rotation_matrix = cv::Mat::eye(3, 3, CV_64F);
    calib_data.translation_vector = cv::Mat::zeros(3, 1, CV_64F);
    
    // 간단한 호모그래피 (기본 탑뷰 변환)
    calib_data.homography_matrix = create_default_homography(camera_id, image_width, image_height);
    
    // 추가 정보 설정
    calib_data.camera_name = CAMERA_NAMES.at(camera_id);
    calib_data.is_fisheye = false;  // 기본적으로 일반 렌즈로 가정
    calib_data.reprojection_error = 0.0;
    
    // 탑뷰 영역 설정 (각 카메라별로 다른 영역)
    calib_data.roi_in_topview = create_default_roi(camera_id, 1024, 1024);  // 기본 출력 크기
    calib_data.ground_center = cv::Point2f(0.0f, 0.0f);
    calib_data.pixels_per_meter = 20.0;  // 1미터당 20픽셀
    
    if (!set_camera_calibration(camera_id, calib_data)) {
      RCLCPP_ERROR(node_->get_logger(), "카메라 %d 기본 캘리브레이션 설정 실패", camera_id);
      return false;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "모든 카메라 기본 캘리브레이션 완료");
  return true;
}

bool CameraCalibrationManager::are_all_cameras_calibrated() const
{
  for (int i = 0; i < NUM_CAMERAS; ++i) {
    if (get_camera_calibration(i) == nullptr) {
      return false;
    }
  }
  return true;
}

cv::Mat CameraCalibrationManager::undistort_image(int camera_id, const cv::Mat& distorted_image) const
{
  const CameraCalibrationData* calib = get_camera_calibration(camera_id);
  if (!calib) {
    RCLCPP_WARN(node_->get_logger(), "카메라 %d 캘리브레이션 데이터 없음", camera_id);
    return distorted_image.clone();  // 원본 반환
  }

  cv::Mat undistorted;
  if (calib->is_fisheye) {
    // 어안 렌즈 언디스토트
    cv::fisheye::undistortImage(distorted_image, undistorted, 
                               calib->camera_matrix, calib->distortion_coefficients);
  } else {
    // 일반 렌즈 언디스토트
    cv::undistort(distorted_image, undistorted, 
                  calib->camera_matrix, calib->distortion_coefficients);
  }

  return undistorted;
}

cv::Mat CameraCalibrationManager::transform_to_topview(
  int camera_id, 
  const cv::Mat& input_image, 
  const cv::Size& output_size) const
{
  const CameraCalibrationData* calib = get_camera_calibration(camera_id);
  if (!calib) {
    RCLCPP_WARN(node_->get_logger(), "카메라 %d 캘리브레이션 데이터 없음", camera_id);
    return cv::Mat();
  }

  cv::Mat topview;
  cv::warpPerspective(input_image, topview, calib->homography_matrix, output_size);
  
  return topview;
}

void CameraCalibrationManager::print_calibration_summary() const
{
  RCLCPP_INFO(node_->get_logger(), "=== 캘리브레이션 요약 ===");
  
  for (int i = 0; i < NUM_CAMERAS; ++i) {
    const CameraCalibrationData* calib = get_camera_calibration(i);
    if (calib) {
      RCLCPP_INFO(node_->get_logger(), "카메라 %d (%s): 완료 (RMS 오차: %.3f)", 
                  i, calib->camera_name.c_str(), calib->reprojection_error);
    } else {
      RCLCPP_INFO(node_->get_logger(), "카메라 %d: 미완료", i);
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "========================");
}

// =============== Private 함수들 ===============

bool CameraCalibrationManager::validate_calibration_data(const CameraCalibrationData& data) const
{
  // 카메라 행렬 검증
  if (data.camera_matrix.rows != 3 || data.camera_matrix.cols != 3) {
    return false;
  }

  // 이미지 크기 검증
  if (data.image_size.width <= 0 || data.image_size.height <= 0) {
    return false;
  }

  // 왜곡 계수 검증
  if (data.distortion_coefficients.empty()) {
    return false;
  }

  return true;
}

cv::Mat CameraCalibrationManager::create_camera_matrix(double fx, double fy, double cx, double cy) const
{
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fy;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;
  return K;
}

cv::Mat CameraCalibrationManager::create_distortion_coefficients(double k1, double k2, double k3, double k4) const
{
  cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
  dist_coeffs.at<double>(0) = k1;
  dist_coeffs.at<double>(1) = k2;
  dist_coeffs.at<double>(2) = k3;
  dist_coeffs.at<double>(3) = k4;
  return dist_coeffs;
}

cv::Mat CameraCalibrationManager::create_default_homography(int camera_id, int image_width, int image_height) const
{
  // 간단한 기본 호모그래피 생성 (실제로는 더 정교한 계산 필요)
  cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
  
  // 카메라별로 다른 변환 적용 (예시)
  switch (camera_id) {
    case 0:  // front
      // 전방 카메라는 약간의 회전과 스케일링
      H.at<double>(0, 0) = 0.5;
      H.at<double>(1, 1) = 0.8;
      H.at<double>(0, 2) = image_width * 0.25;
      H.at<double>(1, 2) = image_height * 0.1;
      break;
    case 1:  // back
      // 후방 카메라는 180도 회전
      H.at<double>(0, 0) = -0.5;
      H.at<double>(1, 1) = -0.8;
      H.at<double>(0, 2) = image_width * 0.75;
      H.at<double>(1, 2) = image_height * 0.9;
      break;
    case 2:  // left
      // 좌측 카메라는 90도 회전
      H.at<double>(0, 0) = 0.0;
      H.at<double>(0, 1) = -0.8;
      H.at<double>(1, 0) = 0.5;
      H.at<double>(1, 1) = 0.0;
      H.at<double>(0, 2) = image_width * 0.1;
      H.at<double>(1, 2) = image_height * 0.5;
      break;
    case 3:  // right
      // 우측 카메라는 -90도 회전
      H.at<double>(0, 0) = 0.0;
      H.at<double>(0, 1) = 0.8;
      H.at<double>(1, 0) = -0.5;
      H.at<double>(1, 1) = 0.0;
      H.at<double>(0, 2) = image_width * 0.9;
      H.at<double>(1, 2) = image_height * 0.5;
      break;
  }
  
  return H;
}

cv::Rect CameraCalibrationManager::create_default_roi(int camera_id, int output_width, int output_height) const
{
  // 각 카메라별 기본 ROI 영역 설정
  int quarter_w = output_width / 2;
  int quarter_h = output_height / 2;
  
  switch (camera_id) {
    case 0:  // front - 상단 중앙
      return cv::Rect(quarter_w / 2, 0, quarter_w, quarter_h);
    case 1:  // back - 하단 중앙
      return cv::Rect(quarter_w / 2, quarter_h, quarter_w, quarter_h);
    case 2:  // left - 좌측 중앙
      return cv::Rect(0, quarter_h / 2, quarter_w, quarter_h);
    case 3:  // right - 우측 중앙
      return cv::Rect(quarter_w, quarter_h / 2, quarter_w, quarter_h);
    default:
      return cv::Rect(0, 0, quarter_w, quarter_h);
  }
}

bool CameraCalibrationManager::calibrate_camera_from_chessboard(
  int camera_id,
  const std::vector<cv::Mat>& calibration_images,
  const cv::Size& board_size,
  float square_size)
{
  // TODO: 체스보드 기반 캘리브레이션 구현
  RCLCPP_WARN(node_->get_logger(), "체스보드 캘리브레이션 미구현");
  return false;
}

bool CameraCalibrationManager::calculate_ground_homography(
  int camera_id,
  const std::vector<cv::Point2f>& ground_points,
  const std::vector<cv::Point2f>& image_points)
{
  // TODO: 지면 기반 호모그래피 계산 구현
  RCLCPP_WARN(node_->get_logger(), "지면 호모그래피 계산 미구현");
  return false;
}

double CameraCalibrationManager::validate_calibration_quality(int camera_id, const std::vector<cv::Mat>& test_images)
{
  // TODO: 캘리브레이션 품질 검증 구현
  RCLCPP_WARN(node_->get_logger(), "캘리브레이션 품질 검증 미구현");
  return 0.0;
}

bool CameraCalibrationManager::save_calibration_to_file(const std::string& output_file_path) const
{
  // TODO: YAML 파일로 저장 구현
  RCLCPP_WARN(node_->get_logger(), "캘리브레이션 저장 미구현");
  return false;
}

}  // namespace econ_surround_view 