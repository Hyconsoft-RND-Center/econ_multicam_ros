#include "econ_surround_view/surround_view_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace econ_surround_view
{

SurroundViewNode::SurroundViewNode(const rclcpp::NodeOptions & options)
: Node("surround_view_node", options),
  front_sub_(this, ""),
  back_sub_(this, ""),
  left_sub_(this, ""),
  right_sub_(this, ""),
  frame_count_(0),
  avg_processing_time_ms_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "서라운드 뷰 노드를 초기화합니다...");

  // 파라미터 선언 및 로드
  declare_parameters();
  load_parameters();

  // Image transport 초기화
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  // QoS 설정 (BestEffort)
  auto qos = rclcpp::QoS(10).best_effort();

  // 구독자 초기화
  front_sub_.subscribe(this, front_camera_topic_, qos.get_rmw_qos_profile());
  back_sub_.subscribe(this, back_camera_topic_, qos.get_rmw_qos_profile());
  left_sub_.subscribe(this, left_camera_topic_, qos.get_rmw_qos_profile());
  right_sub_.subscribe(this, right_camera_topic_, qos.get_rmw_qos_profile());

  // 시간 동기화 설정 (100ms 내외 차이 허용)
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), front_sub_, back_sub_, left_sub_, right_sub_);
  // Note: ROS2 message_filters에서는 setAgeMsgTolerance 메서드가 다르거나 없을 수 있음
  
  sync_->registerCallback(
    std::bind(&SurroundViewNode::camera_sync_callback, this,
              std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3, std::placeholders::_4));

  // 퍼블리셔 초기화
  surround_view_pub_ = it_->advertise(output_topic_, 1);

  // 처리 클래스들 초기화
  vpi_processor_ = std::make_unique<VPIImageProcessor>(
    1920, 1080,  // 입력 이미지 크기 (기본값)
    output_width_, output_height_);

  calib_manager_ = std::make_unique<CameraCalibrationManager>(this);
  stitcher_ = std::make_unique<SurroundViewStitcher>(output_width_, output_height_, this);

  // VPI 프로세서 초기화 확인
  if (!vpi_processor_->is_initialized()) {
    RCLCPP_ERROR(this->get_logger(), "VPI 프로세서 초기화 실패!");
    return;
  }

  // 기본 캘리브레이션 로드 시도
  std::string calib_file = this->declare_parameter("calibration_file", std::string(""));
  if (!calib_file.empty()) {
    if (calib_manager_->load_calibration_from_file(calib_file)) {
      RCLCPP_INFO(this->get_logger(), "캘리브레이션 파일 로드 성공: %s", calib_file.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "캘리브레이션 파일 로드 실패. 기본값 사용: %s", calib_file.c_str());
      calib_manager_->create_default_calibration(1920, 1080);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "캘리브레이션 파일이 지정되지 않음. 기본값 사용");
    calib_manager_->create_default_calibration(1920, 1080);
  }

  // 스티처 기본 레이아웃 설정
  if (!stitcher_->setup_default_layout("cross")) {
    RCLCPP_ERROR(this->get_logger(), "스티처 기본 레이아웃 설정 실패!");
    return;
  }

  // 블렌딩 방법 설정
  stitcher_->set_blending_method(BlendingMethod::FEATHERING, blend_alpha_);

  // 차량 오버레이 설정 (선택사항)
  std::string vehicle_model = this->declare_parameter("vehicle_model_path", std::string(""));
  if (!vehicle_model.empty()) {
    cv::Point2f vehicle_center(output_width_ / 2.0f, output_height_ / 2.0f);
    cv::Size vehicle_size(200, 400);  // 기본 차량 크기
    stitcher_->set_vehicle_overlay(vehicle_model, vehicle_center, vehicle_size);
  }

  // 그리드 오버레이 설정
  bool enable_grid = this->declare_parameter("enable_grid_overlay", true);
  int grid_spacing = this->declare_parameter("grid_spacing", 50);
  stitcher_->set_grid_overlay(enable_grid, grid_spacing);

  RCLCPP_INFO(this->get_logger(), "서라운드 뷰 노드 초기화 완료");
  RCLCPP_INFO(this->get_logger(), "  - 출력 해상도: %dx%d", output_width_, output_height_);
  RCLCPP_INFO(this->get_logger(), "  - VPI 가속: %s", enable_vpi_processing_ ? "활성화" : "비활성화");
  RCLCPP_INFO(this->get_logger(), "  - 렌즈 왜곡 보정: %s", enable_lens_distortion_correction_ ? "활성화" : "비활성화");
  RCLCPP_INFO(this->get_logger(), "구독 대기 중...");
}

SurroundViewNode::~SurroundViewNode()
{
  RCLCPP_INFO(this->get_logger(), "서라운드 뷰 노드를 종료합니다...");
  RCLCPP_INFO(this->get_logger(), "처리된 총 프레임: %zu", frame_count_);
  if (frame_count_ > 0) {
    RCLCPP_INFO(this->get_logger(), "평균 처리 시간: %.2f ms", avg_processing_time_ms_);
  }
}

void SurroundViewNode::declare_parameters()
{
  // 카메라 토픽 이름들
  this->declare_parameter("front_camera_topic", "/dev/video0/image_raw");
  this->declare_parameter("back_camera_topic", "/dev/video1/image_raw");
  this->declare_parameter("left_camera_topic", "/dev/video2/image_raw");
  this->declare_parameter("right_camera_topic", "/dev/video3/image_raw");
  this->declare_parameter("output_topic", "/surround_view/image_raw");

  // 처리 파라미터들
  this->declare_parameter("output_width", 1024);
  this->declare_parameter("output_height", 1024);
  this->declare_parameter("enable_vpi_processing", true);
  this->declare_parameter("enable_lens_distortion_correction", true);
  this->declare_parameter("blend_alpha", 0.5);

  // 성능 파라미터들
  this->declare_parameter("max_queue_size", 10);
  this->declare_parameter("processing_threads", 4);
}

void SurroundViewNode::load_parameters()
{
  front_camera_topic_ = this->get_parameter("front_camera_topic").as_string();
  back_camera_topic_ = this->get_parameter("back_camera_topic").as_string();
  left_camera_topic_ = this->get_parameter("left_camera_topic").as_string();
  right_camera_topic_ = this->get_parameter("right_camera_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();

  output_width_ = this->get_parameter("output_width").as_int();
  output_height_ = this->get_parameter("output_height").as_int();
  enable_vpi_processing_ = this->get_parameter("enable_vpi_processing").as_bool();
  enable_lens_distortion_correction_ = this->get_parameter("enable_lens_distortion_correction").as_bool();
  blend_alpha_ = this->get_parameter("blend_alpha").as_double();

  // 파라미터 검증
  if (output_width_ <= 0 || output_height_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "잘못된 출력 크기. 기본값 사용: 1024x1024");
    output_width_ = output_height_ = 1024;
  }

  if (blend_alpha_ < 0.0 || blend_alpha_ > 1.0) {
    RCLCPP_WARN(this->get_logger(), "잘못된 블렌드 알파값. 기본값 사용: 0.5");
    blend_alpha_ = 0.5;
  }
}

void SurroundViewNode::camera_sync_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & front_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & back_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & left_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_img)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  try {
    // ROS 이미지를 OpenCV Mat으로 변환
    cv_bridge::CvImagePtr front_cv, back_cv, left_cv, right_cv;
    
    front_cv = cv_bridge::toCvCopy(front_img, sensor_msgs::image_encodings::BGR8);
    back_cv = cv_bridge::toCvCopy(back_img, sensor_msgs::image_encodings::BGR8);
    left_cv = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::BGR8);
    right_cv = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::BGR8);

    // 이미지 크기 검증
    if (front_cv->image.empty() || back_cv->image.empty() || 
        left_cv->image.empty() || right_cv->image.empty()) {
      RCLCPP_WARN(this->get_logger(), "빈 이미지가 수신됨");
      return;
    }

    // 서라운드 뷰 처리
    process_surround_view(front_cv->image, back_cv->image, 
                         left_cv->image, right_cv->image, front_img->header);

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
    return;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "이미지 처리 예외: %s", e.what());
    return;
  }

  // 성능 통계 업데이트
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  double processing_time_ms = duration.count() / 1000.0;

  frame_count_++;
  avg_processing_time_ms_ = (avg_processing_time_ms_ * (frame_count_ - 1) + processing_time_ms) / frame_count_;

  // 주기적 성능 로깅 (100프레임마다)
  if (frame_count_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), 
                "처리 통계 - 프레임: %zu, 평균 처리시간: %.2f ms, VPI 평균: %.2f ms, 스티칭 평균: %.2f ms",
                frame_count_, avg_processing_time_ms_, 
                vpi_processor_->get_average_processing_time(),
                stitcher_->get_average_stitching_time());
  }
}

void SurroundViewNode::process_surround_view(
  const cv::Mat & front_img,
  const cv::Mat & back_img,
  const cv::Mat & left_img,
  const cv::Mat & right_img,
  const std_msgs::msg::Header & header)
{
  // 입력 이미지 벡터 생성
  std::vector<cv::Mat> input_images = {front_img, back_img, left_img, right_img};

  std::vector<cv::Mat> processed_images;

  if (enable_vpi_processing_) {
    // VPI를 사용한 배치 처리
    processed_images = vpi_processor_->process_camera_images(input_images);
  } else {
    // 기본 OpenCV 처리 (개발/디버깅용)
    processed_images.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
      cv::Mat processed;
      if (enable_lens_distortion_correction_) {
        processed = calib_manager_->undistort_image(i, input_images[i]);
      } else {
        processed = input_images[i].clone();
      }
      
      // 간단한 탑뷰 변환 (실제로는 호모그래피 적용)
      cv::Size output_size(output_width_ / 2, output_height_ / 2);
      processed = calib_manager_->transform_to_topview(i, processed, output_size);
      processed_images.push_back(processed);
    }
  }

  // 처리된 이미지 검증
  if (processed_images.size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "이미지 처리 실패: 예상 4개, 실제 %zu개", processed_images.size());
    return;
  }

  // 서라운드 뷰 스티칭
  cv::Mat surround_view = stitcher_->stitch_images(processed_images);

  if (surround_view.empty()) {
    RCLCPP_ERROR(this->get_logger(), "서라운드 뷰 스티칭 실패");
    return;
  }

  // ROS 이미지 메시지로 변환 및 퍼블리시
  try {
    sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::BGR8, surround_view).toImageMsg();
    
    output_msg->header.frame_id = "surround_view";
    surround_view_pub_.publish(output_msg);

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "출력 이미지 변환 실패: %s", e.what());
  }
}

}  // namespace econ_surround_view

// 메인 함수
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<econ_surround_view::SurroundViewNode>(options);

  // 멀티 스레드 실행기 사용 (성능 향상)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "서라운드 뷰 노드 시작");

  try {
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "실행 중 예외 발생: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
} 