#include "econ_surround_view/surround_view_stitcher.hpp"

#include <chrono>
#include <iostream>
#include <opencv2/imgproc.hpp>

namespace econ_surround_view
{

// 카메라 이름 정의
const std::array<std::string, 4> SurroundViewStitcher::CAMERA_NAMES = {
  "front", "back", "left", "right"
};

SurroundViewStitcher::SurroundViewStitcher(int output_width, int output_height, rclcpp::Node* node)
: output_width_(output_width), output_height_(output_height), node_(node),
  blending_method_(BlendingMethod::FEATHERING), alpha_value_(DEFAULT_ALPHA), 
  blend_width_(DEFAULT_BLEND_WIDTH), enable_color_correction_(false), 
  reference_camera_id_(0), enable_vehicle_overlay_(false), 
  enable_grid_overlay_(false), grid_spacing_(50), 
  grid_color_(cv::Scalar(128, 128, 128)), avg_stitching_time_ms_(0.0), frame_count_(0)
{
  RCLCPP_INFO(node_->get_logger(), "서라운드 뷰 스티처 초기화 (%dx%d)", output_width_, output_height_);

  // 카메라 영역들 초기화
  for (int i = 0; i < NUM_CAMERAS; ++i) {
    camera_regions_[i].is_active = true;
  }

  // 차량 중심 설정
  vehicle_center_ = cv::Point2f(output_width_ / 2.0f, output_height_ / 2.0f);
  vehicle_size_ = cv::Size(100, 200);  // 기본 차량 크기

  RCLCPP_INFO(node_->get_logger(), "서라운드 뷰 스티처 초기화 완료");
}

bool SurroundViewStitcher::set_camera_region(int camera_id, const cv::Rect& roi, int blend_width)
{
  if (camera_id < 0 || camera_id >= NUM_CAMERAS) {
    RCLCPP_ERROR(node_->get_logger(), "잘못된 카메라 ID: %d", camera_id);
    return false;
  }

  camera_regions_[camera_id].roi = roi;
  camera_regions_[camera_id].valid_area = roi;
  blend_width_ = blend_width;

  // 블렌딩 마스크 생성
  create_feathering_mask(camera_id, blend_width);

  RCLCPP_INFO(node_->get_logger(), "카메라 %d (%s) 영역 설정: (%d,%d,%d,%d)", 
              camera_id, CAMERA_NAMES[camera_id].c_str(), 
              roi.x, roi.y, roi.width, roi.height);

  return true;
}

bool SurroundViewStitcher::set_blending_method(BlendingMethod method, double alpha)
{
  blending_method_ = method;
  alpha_value_ = std::clamp(alpha, 0.0, 1.0);

  RCLCPP_INFO(node_->get_logger(), "블렌딩 방법 설정: %d, 알파: %.2f", 
              static_cast<int>(method), alpha_value_);

  return true;
}

cv::Mat SurroundViewStitcher::stitch_images(const std::vector<cv::Mat>& camera_images)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  if (!validate_camera_images(camera_images)) {
    RCLCPP_ERROR(node_->get_logger(), "입력 이미지 검증 실패");
    return cv::Mat();
  }

  // 출력 이미지 초기화
  cv::Mat surround_view = cv::Mat::zeros(output_height_, output_width_, CV_8UC3);

  try {
    // 각 카메라 이미지를 해당 영역에 배치
    for (int i = 0; i < NUM_CAMERAS; ++i) {
      if (!camera_regions_[i].is_active || camera_images[i].empty()) {
        continue;
      }

      // 이미지 크기를 ROI에 맞게 리사이즈
      cv::Mat resized_image;
      cv::resize(camera_images[i], resized_image, camera_regions_[i].roi.size());

      // 색상 보정 적용 (옵션)
      if (enable_color_correction_) {
        resized_image = apply_color_correction(resized_image, i);
      }

      // ROI 영역에 이미지 복사
      place_image_in_roi(surround_view, resized_image, camera_regions_[i].roi, i);
    }

    // 블렌딩 처리
    apply_blending(surround_view, camera_images);

    // 오버레이 그리기
    if (enable_grid_overlay_) {
      draw_grid_overlay(surround_view);
    }

    if (enable_vehicle_overlay_) {
      draw_vehicle_overlay(surround_view);
    }

    // 카메라 라벨 그리기 (디버깅용)
    draw_camera_labels(surround_view);

  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "OpenCV 예외: %s", e.what());
    return cv::Mat();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "스티칭 중 예외: %s", e.what());
    return cv::Mat();
  }

  // 성능 통계 업데이트
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  double stitching_time_ms = duration.count() / 1000.0;

  stitching_times_.push_back(stitching_time_ms);
  if (stitching_times_.size() > 100) {  // 최근 100개만 유지
    stitching_times_.erase(stitching_times_.begin());
  }

  // 평균 계산
  double sum = 0.0;
  for (double time : stitching_times_) {
    sum += time;
  }
  avg_stitching_time_ms_ = sum / stitching_times_.size();
  frame_count_++;

  return surround_view;
}

bool SurroundViewStitcher::setup_default_layout(const std::string& layout_type)
{
  RCLCPP_INFO(node_->get_logger(), "기본 레이아웃 설정: %s", layout_type.c_str());

  if (layout_type == "cross") {
    // 십자형 레이아웃 (일반적인 서라운드 뷰)
    int quarter_w = output_width_ / 2;
    int quarter_h = output_height_ / 2;

    // 카메라별 영역 설정
    set_camera_region(0, cv::Rect(quarter_w / 2, 0, quarter_w, quarter_h));          // front - 상단
    set_camera_region(1, cv::Rect(quarter_w / 2, quarter_h, quarter_w, quarter_h));  // back - 하단
    set_camera_region(2, cv::Rect(0, quarter_h / 2, quarter_w, quarter_h));         // left - 좌측
    set_camera_region(3, cv::Rect(quarter_w, quarter_h / 2, quarter_w, quarter_h)); // right - 우측

  } else if (layout_type == "square") {
    // 사각형 레이아웃 (2x2 그리드)
    int half_w = output_width_ / 2;
    int half_h = output_height_ / 2;

    set_camera_region(0, cv::Rect(0, 0, half_w, half_h));              // front - 좌상단
    set_camera_region(1, cv::Rect(half_w, half_h, half_w, half_h));    // back - 우하단
    set_camera_region(2, cv::Rect(0, half_h, half_w, half_h));         // left - 좌하단
    set_camera_region(3, cv::Rect(half_w, 0, half_w, half_h));         // right - 우상단

  } else {
    RCLCPP_WARN(node_->get_logger(), "알 수 없는 레이아웃 타입: %s. cross 레이아웃 사용", layout_type.c_str());
    return setup_default_layout("cross");
  }

  // 블렌딩 마스크 초기화
  initialize_blend_masks();

  return true;
}

bool SurroundViewStitcher::set_grid_overlay(bool enable_grid, int grid_spacing, const cv::Scalar& grid_color)
{
  enable_grid_overlay_ = enable_grid;
  grid_spacing_ = grid_spacing;
  grid_color_ = grid_color;

  RCLCPP_INFO(node_->get_logger(), "그리드 오버레이: %s, 간격: %d", 
              enable_grid ? "활성화" : "비활성화", grid_spacing);

  return true;
}

void SurroundViewStitcher::set_camera_active(int camera_id, bool active)
{
  if (camera_id >= 0 && camera_id < NUM_CAMERAS) {
    camera_regions_[camera_id].is_active = active;
    RCLCPP_INFO(node_->get_logger(), "카메라 %d (%s): %s", 
                camera_id, CAMERA_NAMES[camera_id].c_str(),
                active ? "활성화" : "비활성화");
  }
}

void SurroundViewStitcher::print_configuration_summary() const
{
  RCLCPP_INFO(node_->get_logger(), "=== 서라운드 뷰 스티처 설정 ===");
  RCLCPP_INFO(node_->get_logger(), "출력 크기: %dx%d", output_width_, output_height_);
  RCLCPP_INFO(node_->get_logger(), "블렌딩 방법: %d", static_cast<int>(blending_method_));
  RCLCPP_INFO(node_->get_logger(), "블렌딩 알파: %.2f", alpha_value_);
  RCLCPP_INFO(node_->get_logger(), "색상 보정: %s", enable_color_correction_ ? "활성화" : "비활성화");
  RCLCPP_INFO(node_->get_logger(), "그리드 오버레이: %s", enable_grid_overlay_ ? "활성화" : "비활성화");
  RCLCPP_INFO(node_->get_logger(), "차량 오버레이: %s", enable_vehicle_overlay_ ? "활성화" : "비활성화");

  for (int i = 0; i < NUM_CAMERAS; ++i) {
    const auto& region = camera_regions_[i];
    RCLCPP_INFO(node_->get_logger(), "카메라 %d (%s): %s, ROI: (%d,%d,%d,%d)", 
                i, CAMERA_NAMES[i].c_str(),
                region.is_active ? "활성화" : "비활성화",
                region.roi.x, region.roi.y, region.roi.width, region.roi.height);
  }

  if (frame_count_ > 0) {
    RCLCPP_INFO(node_->get_logger(), "평균 스티칭 시간: %.2f ms", avg_stitching_time_ms_);
  }

  RCLCPP_INFO(node_->get_logger(), "==============================");
}

// =============== Private 함수들 ===============

bool SurroundViewStitcher::validate_camera_images(const std::vector<cv::Mat>& images)
{
  if (images.size() != NUM_CAMERAS) {
    RCLCPP_ERROR(node_->get_logger(), "이미지 개수 불일치: 예상 %d, 실제 %zu", NUM_CAMERAS, images.size());
    return false;
  }

  for (size_t i = 0; i < images.size(); ++i) {
    if (camera_regions_[i].is_active && images[i].empty()) {
      RCLCPP_WARN(node_->get_logger(), "카메라 %zu 이미지가 비어있음", i);
    }
  }

  return true;
}

void SurroundViewStitcher::place_image_in_roi(cv::Mat& target, const cv::Mat& source, const cv::Rect& roi, int camera_id)
{
  // ROI 영역이 출력 이미지 범위 내에 있는지 확인
  cv::Rect safe_roi = roi & cv::Rect(0, 0, target.cols, target.rows);
  if (safe_roi.area() == 0) {
    return;
  }

  // 소스 이미지 크기 조정
  cv::Mat resized_source;
  if (source.size() != safe_roi.size()) {
    cv::resize(source, resized_source, safe_roi.size());
  } else {
    resized_source = source;
  }

  // ROI 영역에 복사
  resized_source.copyTo(target(safe_roi));
}

void SurroundViewStitcher::apply_blending(cv::Mat& surround_view, const std::vector<cv::Mat>& camera_images)
{
  // TODO: 실제 블렌딩 구현
  // 현재는 간단한 오버레이만 수행
  switch (blending_method_) {
    case BlendingMethod::SIMPLE_ALPHA:
      // 간단한 알파 블렌딩
      break;
    case BlendingMethod::FEATHERING:
      // 페더링 블렌딩
      break;
    case BlendingMethod::WEIGHTED_AVERAGE:
      // 가중 평균 블렌딩
      break;
    default:
      break;
  }
}

cv::Mat SurroundViewStitcher::apply_color_correction(const cv::Mat& image, int camera_id)
{
  // TODO: 실제 색상 보정 구현
  // 현재는 원본 반환
  return image.clone();
}

void SurroundViewStitcher::draw_grid_overlay(cv::Mat& output_image)
{
  if (!enable_grid_overlay_ || grid_spacing_ <= 0) {
    return;
  }

  // 세로선 그리기
  for (int x = 0; x < output_image.cols; x += grid_spacing_) {
    cv::line(output_image, cv::Point(x, 0), cv::Point(x, output_image.rows - 1), grid_color_, 1);
  }

  // 가로선 그리기
  for (int y = 0; y < output_image.rows; y += grid_spacing_) {
    cv::line(output_image, cv::Point(0, y), cv::Point(output_image.cols - 1, y), grid_color_, 1);
  }

  // 중심점 표시
  int center_x = output_image.cols / 2;
  int center_y = output_image.rows / 2;
  cv::circle(output_image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 0), -1);
}

void SurroundViewStitcher::draw_vehicle_overlay(cv::Mat& output_image)
{
  if (!enable_vehicle_overlay_) {
    return;
  }

  // 간단한 차량 모양 그리기 (사각형)
  cv::Rect vehicle_rect(
    static_cast<int>(vehicle_center_.x - vehicle_size_.width / 2),
    static_cast<int>(vehicle_center_.y - vehicle_size_.height / 2),
    vehicle_size_.width,
    vehicle_size_.height
  );

  cv::rectangle(output_image, vehicle_rect, cv::Scalar(255, 255, 255), 2);
  
  // 차량 방향 표시 (전방)
  cv::Point front_center(
    static_cast<int>(vehicle_center_.x),
    vehicle_rect.y
  );
  cv::circle(output_image, front_center, 8, cv::Scalar(0, 255, 0), -1);
}

void SurroundViewStitcher::draw_camera_labels(cv::Mat& output_image)
{
  for (int i = 0; i < NUM_CAMERAS; ++i) {
    if (!camera_regions_[i].is_active) {
      continue;
    }

    cv::Point label_pos(
      camera_regions_[i].roi.x + 10,
      camera_regions_[i].roi.y + 25
    );

    cv::putText(output_image, CAMERA_NAMES[i], label_pos, 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
  }
}

void SurroundViewStitcher::initialize_blend_masks()
{
  RCLCPP_INFO(node_->get_logger(), "블렌딩 마스크 초기화");
  
  for (int i = 0; i < NUM_CAMERAS; ++i) {
    create_feathering_mask(i, blend_width_);
  }
}

void SurroundViewStitcher::create_feathering_mask(int camera_id, int blend_width)
{
  if (camera_id < 0 || camera_id >= NUM_CAMERAS) {
    return;
  }

  const cv::Rect& roi = camera_regions_[camera_id].roi;
  
  // 기본 마스크 생성 (전체 ROI 영역을 255로 설정)
  camera_regions_[camera_id].blend_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
  camera_regions_[camera_id].blend_mask.setTo(255);

  // 가중치 마스크 생성 (실제 블렌딩에서 사용)
  camera_regions_[camera_id].weight_mask = cv::Mat::ones(roi.size(), CV_32F);

  // TODO: 실제 페더링 마스크 생성 구현
  // 현재는 기본 마스크만 생성
}

cv::Rect SurroundViewStitcher::calculate_overlap_region(const cv::Rect& roi1, const cv::Rect& roi2)
{
  return roi1 & roi2;  // OpenCV의 교집합 연산
}

bool SurroundViewStitcher::set_vehicle_overlay(
  const std::string& vehicle_model_path,
  const cv::Point2f& vehicle_center,
  const cv::Size& vehicle_size)
{
  // TODO: 차량 모델 이미지 로드 구현
  enable_vehicle_overlay_ = true;
  vehicle_center_ = vehicle_center;
  vehicle_size_ = vehicle_size;

  RCLCPP_INFO(node_->get_logger(), "차량 오버레이 설정: 중심(%.1f, %.1f), 크기(%d, %d)",
              vehicle_center.x, vehicle_center.y, vehicle_size.width, vehicle_size.height);

  return true;
}

bool SurroundViewStitcher::set_color_correction(bool enable_color_correction, int reference_camera_id)
{
  enable_color_correction_ = enable_color_correction;
  reference_camera_id_ = reference_camera_id;

  RCLCPP_INFO(node_->get_logger(), "색상 보정: %s, 기준 카메라: %d",
              enable_color_correction ? "활성화" : "비활성화", reference_camera_id);

  return true;
}

bool SurroundViewStitcher::adjust_camera_region(int camera_id, const cv::Rect& new_roi)
{
  return set_camera_region(camera_id, new_roi, blend_width_);
}

}  // namespace econ_surround_view 