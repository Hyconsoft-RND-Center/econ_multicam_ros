#ifndef ECON_SURROUND_VIEW__SURROUND_VIEW_NODE_HPP_
#define ECON_SURROUND_VIEW__SURROUND_VIEW_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <vpi/VPI.h>

#include "econ_surround_view/vpi_image_processor.hpp"
#include "econ_surround_view/camera_calibration_manager.hpp"
#include "econ_surround_view/surround_view_stitcher.hpp"

namespace econ_surround_view
{

class SurroundViewNode : public rclcpp::Node, public std::enable_shared_from_this<SurroundViewNode>
{
public:
  explicit SurroundViewNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SurroundViewNode();

private:
  // 콜백 함수
  void camera_sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & front_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & back_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & left_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_img);

  void process_surround_view(
    const cv::Mat & front_img,
    const cv::Mat & back_img,
    const cv::Mat & left_img,
    const cv::Mat & right_img,
    const std_msgs::msg::Header & header);

  // 파라미터 선언 및 로드
  void declare_parameters();
  void load_parameters();

  // 카메라 토픽 이름들
  std::string front_camera_topic_;
  std::string back_camera_topic_;
  std::string left_camera_topic_;
  std::string right_camera_topic_;
  std::string output_topic_;

  // 처리 파라미터들
  int output_width_;
  int output_height_;
  bool enable_vpi_processing_;
  bool enable_lens_distortion_correction_;
  double blend_alpha_;

  // ROS2 구독자들 (message_filters 사용)
  message_filters::Subscriber<sensor_msgs::msg::Image> front_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> back_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;

  // 시간 동기화 정책
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::Image>;
  
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 퍼블리셔
  image_transport::Publisher surround_view_pub_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  // 처리 클래스들
  std::unique_ptr<VPIImageProcessor> vpi_processor_;
  std::unique_ptr<CameraCalibrationManager> calib_manager_;
  std::unique_ptr<SurroundViewStitcher> stitcher_;

  // 통계 및 성능 모니터링
  rclcpp::Time last_process_time_;
  size_t frame_count_;
  double avg_processing_time_ms_;
};

}  // namespace econ_surround_view

#endif  // ECON_SURROUND_VIEW__SURROUND_VIEW_NODE_HPP_ 