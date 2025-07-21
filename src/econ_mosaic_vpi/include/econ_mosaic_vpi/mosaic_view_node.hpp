#ifndef econ_mosaic_vpi_MOSAIC_VIEW_NODE_HPP
#define econ_mosaic_vpi_MOSAIC_VIEW_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <vpi/VPI.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>

class MosaicViewNode : public rclcpp::Node
{
public:
    MosaicViewNode();
    ~MosaicViewNode();

private:
    // 초기화 함수들
    void initializeVPIImages();
    
    // 콜백 함수
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& front_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& rear_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);
    
    // VPI 관련 함수들
    void uploadToVPI(const cv::Mat& cv_image, VPIImage vpi_image);
    void downloadFromVPI(VPIImage vpi_image, cv::Mat& cv_image);
    void processSurroundView();
    void compositeMosaicView();
    void CHECK_STATUS(VPIStatus status);
    
    // ROS2 멤버 변수들
    message_filters::Subscriber<sensor_msgs::msg::Image> front_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rear_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mosaic_pub_;
    
    // VPI 멤버 변수들
    VPIStream stream_ = nullptr;
    VPIImage front_vpi_ = nullptr;
    VPIImage rear_vpi_ = nullptr;
    VPIImage left_vpi_ = nullptr;
    VPIImage right_vpi_ = nullptr;
    VPIImage output_vpi_ = nullptr;
    VPIImage front_resized_ = nullptr;
    VPIImage rear_resized_ = nullptr;
    VPIImage left_resized_ = nullptr;
    VPIImage right_resized_ = nullptr;
    
    // 파라미터 변수들
    int output_width_;
    int output_height_;
    int camera_width_;
    int camera_height_;
};

#endif // econ_mosaic_vpi_MOSAIC_VIEW_NODE_HPP 