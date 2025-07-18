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
    MosaicViewNode() : Node("mosaic_view_node")
    {
        // Initialize VPI with CUDA and CPU backends
        CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA | VPI_BACKEND_CPU, &stream_));
        
        // Parameters
        this->declare_parameter("output_width", 800);
        this->declare_parameter("output_height", 600);
        this->declare_parameter("camera_width", 1920);
        this->declare_parameter("camera_height", 1080);
        
        output_width_ = this->get_parameter("output_width").as_int();
        output_height_ = this->get_parameter("output_height").as_int();
        camera_width_ = this->get_parameter("camera_width").as_int();
        camera_height_ = this->get_parameter("camera_height").as_int();
        
        // Initialize VPI images
        initializeVPIImages();
        
        // Set QoS profile for camera topics (BEST_EFFORT to match camera drivers)
        rclcpp::QoS camera_qos(10);
        camera_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        // Create subscribers for 4 cameras with appropriate QoS
        front_sub_.subscribe(this, "/dev/video0/image_raw", camera_qos.get_rmw_qos_profile());
        rear_sub_.subscribe(this, "/dev/video1/image_raw", camera_qos.get_rmw_qos_profile());
        left_sub_.subscribe(this, "/dev/video2/image_raw", camera_qos.get_rmw_qos_profile());
        right_sub_.subscribe(this, "/dev/video3/image_raw", camera_qos.get_rmw_qos_profile());
        
        // Synchronize messages
        sync_.reset(new Sync(SyncPolicy(10), front_sub_, rear_sub_, left_sub_, right_sub_));
        sync_->registerCallback(std::bind(&MosaicViewNode::imageCallback, this, 
                                         std::placeholders::_1, std::placeholders::_2,
                                         std::placeholders::_3, std::placeholders::_4));
        
        // Create publisher using regular ROS2 publisher instead of image_transport
        mosaic_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mosaic_view/image_raw", 1);
        
        RCLCPP_INFO(this->get_logger(), "Mosaic View Node initialized with VPI acceleration");
    }
    
    ~MosaicViewNode()
    {
        // Cleanup VPI resources
        if (stream_ != nullptr) vpiStreamDestroy(stream_);
        if (front_vpi_ != nullptr) vpiImageDestroy(front_vpi_);
        if (rear_vpi_ != nullptr) vpiImageDestroy(rear_vpi_);
        if (left_vpi_ != nullptr) vpiImageDestroy(left_vpi_);
        if (right_vpi_ != nullptr) vpiImageDestroy(right_vpi_);
        if (output_vpi_ != nullptr) vpiImageDestroy(output_vpi_);
        if (front_resized_ != nullptr) vpiImageDestroy(front_resized_);
        if (rear_resized_ != nullptr) vpiImageDestroy(rear_resized_);
        if (left_resized_ != nullptr) vpiImageDestroy(left_resized_);
        if (right_resized_ != nullptr) vpiImageDestroy(right_resized_);
    }

private:
    void initializeVPIImages()
    {
        // Create VPI images for input cameras - use both CUDA and CPU backends
        // CPU backend is needed for mapping images to host memory for OpenCV conversion
        uint64_t backend_flags = VPI_BACKEND_CUDA | VPI_BACKEND_CPU;
        
        CHECK_STATUS(vpiImageCreate(camera_width_, camera_height_, VPI_IMAGE_FORMAT_BGR8, backend_flags, &front_vpi_));
        CHECK_STATUS(vpiImageCreate(camera_width_, camera_height_, VPI_IMAGE_FORMAT_BGR8, backend_flags, &rear_vpi_));
        CHECK_STATUS(vpiImageCreate(camera_width_, camera_height_, VPI_IMAGE_FORMAT_BGR8, backend_flags, &left_vpi_));
        CHECK_STATUS(vpiImageCreate(camera_width_, camera_height_, VPI_IMAGE_FORMAT_BGR8, backend_flags, &right_vpi_));
        
        // Create resized images for each camera (quarter size for mosaic view)
        int quarter_width = output_width_ / 2;
        int quarter_height = output_height_ / 2;
        
        CHECK_STATUS(vpiImageCreate(quarter_width, quarter_height, VPI_IMAGE_FORMAT_BGR8, backend_flags, &front_resized_));
        CHECK_STATUS(vpiImageCreate(quarter_width, quarter_height, VPI_IMAGE_FORMAT_BGR8, backend_flags, &rear_resized_));
        CHECK_STATUS(vpiImageCreate(quarter_width, quarter_height, VPI_IMAGE_FORMAT_BGR8, backend_flags, &left_resized_));
        CHECK_STATUS(vpiImageCreate(quarter_width, quarter_height, VPI_IMAGE_FORMAT_BGR8, backend_flags, &right_resized_));
        
        // Create output mosaic view image
        CHECK_STATUS(vpiImageCreate(output_width_, output_height_, VPI_IMAGE_FORMAT_BGR8, backend_flags, &output_vpi_));
        
        RCLCPP_INFO(this->get_logger(), "VPI images initialized - Camera: %dx%d, Output: %dx%d", 
                   camera_width_, camera_height_, output_width_, output_height_);
    }
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& front_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& rear_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        try {
            // Validate input image dimensions
            if (front_msg->width != camera_width_ || front_msg->height != camera_height_) {
                RCLCPP_WARN(this->get_logger(), "Front camera image size mismatch: expected %dx%d, got %dx%d", 
                           camera_width_, camera_height_, front_msg->width, front_msg->height);
                return;
            }
            
            // Convert ROS images to OpenCV
            cv_bridge::CvImagePtr front_cv = cv_bridge::toCvCopy(front_msg, "bgr8");
            cv_bridge::CvImagePtr rear_cv = cv_bridge::toCvCopy(rear_msg, "bgr8");
            cv_bridge::CvImagePtr left_cv = cv_bridge::toCvCopy(left_msg, "bgr8");
            cv_bridge::CvImagePtr right_cv = cv_bridge::toCvCopy(right_msg, "bgr8");
            
            // Validate OpenCV images
            if (front_cv->image.empty() || rear_cv->image.empty() || 
                left_cv->image.empty() || right_cv->image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "One or more camera images are empty");
                return;
            }
            
            // Upload OpenCV images to VPI
            uploadToVPI(front_cv->image, front_vpi_);
            uploadToVPI(rear_cv->image, rear_vpi_);
            uploadToVPI(left_cv->image, left_vpi_);
            uploadToVPI(right_cv->image, right_vpi_);
            
            // Process images with VPI
            processSurroundView();
            
            // Download result and publish
            cv::Mat result_cv;
            downloadFromVPI(output_vpi_, result_cv);
            
            // Validate result image
            if (result_cv.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create mosaic view image");
                return;
            }
            
            // Convert back to ROS message and publish
            sensor_msgs::msg::Image::SharedPtr output_msg = 
                cv_bridge::CvImage(front_msg->header, "bgr8", result_cv).toImageMsg();
            output_msg->header.stamp = this->now();
            mosaic_pub_->publish(*output_msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing mosaic view: %s", e.what());
        }
    }
    
    void uploadToVPI(const cv::Mat& cv_image, VPIImage vpi_image)
    {
        // Validate input image
        if (cv_image.empty() || cv_image.cols <= 0 || cv_image.rows <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid input image for VPI upload");
            return;
        }
        
        VPIImageData img_data;
        CHECK_STATUS(vpiImageLockData(vpi_image, VPI_LOCK_WRITE, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &img_data));
        
        VPIImageBufferPitchLinear& host_data = img_data.buffer.pitch;
        
        // Ensure we have the correct number of planes for BGR8 format
        if (host_data.numPlanes < 1) {
            RCLCPP_ERROR(this->get_logger(), "Invalid number of planes in VPI image");
            vpiImageUnlock(vpi_image);
            return;
        }
        
        // Copy data from OpenCV to VPI
        for (int y = 0; y < cv_image.rows; ++y) {
            const uint8_t* src_row = cv_image.ptr<uint8_t>(y);
            uint8_t* dst_row = reinterpret_cast<uint8_t*>(host_data.planes[0].data) + y * host_data.planes[0].pitchBytes;
            std::memcpy(dst_row, src_row, cv_image.cols * cv_image.channels());
        }
        
        CHECK_STATUS(vpiImageUnlock(vpi_image));
    }
    
    void downloadFromVPI(VPIImage vpi_image, cv::Mat& cv_image)
    {
        static bool first_call = true;
        
        // Get VPI image information first
        int32_t vpi_width, vpi_height;
        VPIImageFormat vpi_format;
        CHECK_STATUS(vpiImageGetSize(vpi_image, &vpi_width, &vpi_height));
        CHECK_STATUS(vpiImageGetFormat(vpi_image, &vpi_format));
        
        if (first_call) {
            RCLCPP_INFO(this->get_logger(), "VPI image info: %dx%d, format: %d", vpi_width, vpi_height, vpi_format);
            first_call = false;
        }
        
        VPIImageData img_data;
        CHECK_STATUS(vpiImageLockData(vpi_image, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &img_data));
        
        VPIImageBufferPitchLinear& host_data = img_data.buffer.pitch;
        
        // Use actual VPI image dimensions instead of expected dimensions
        if (vpi_width <= 0 || vpi_height <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid VPI image dimensions: %dx%d", vpi_width, vpi_height);
            vpiImageUnlock(vpi_image);
            return;
        }
        
        cv_image.create(vpi_height, vpi_width, CV_8UC3);
        
        // Copy data from VPI to OpenCV using actual dimensions
        for (int y = 0; y < vpi_height; ++y) {
            const uint8_t* src_row = reinterpret_cast<const uint8_t*>(host_data.planes[0].data) + y * host_data.planes[0].pitchBytes;
            uint8_t* dst_row = cv_image.ptr<uint8_t>(y);
            std::memcpy(dst_row, src_row, vpi_width * 3);
        }
        
        CHECK_STATUS(vpiImageUnlock(vpi_image));
    }
    
    void processSurroundView()
    {
        // Resize each camera image to quarter size using VPI
        CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, front_vpi_, front_resized_, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
        CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, rear_vpi_, rear_resized_, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
        CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, left_vpi_, left_resized_, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
        CHECK_STATUS(vpiSubmitRescale(stream_, VPI_BACKEND_CUDA, right_vpi_, right_resized_, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
        
        // Wait for rescaling to complete
        CHECK_STATUS(vpiStreamSync(stream_));
        
        // Composite mosaic view (simple 2x2 grid layout)
        compositeMosaicView();
    }
    
    void compositeMosaicView()
    {
        static bool first_call = true;
        
        int quarter_width = output_width_ / 2;
        int quarter_height = output_height_ / 2;
        
        // Download resized images to create composite
        cv::Mat front_mat, rear_mat, left_mat, right_mat;
        downloadFromVPI(front_resized_, front_mat);
        downloadFromVPI(rear_resized_, rear_mat);
        downloadFromVPI(left_resized_, left_mat);
        downloadFromVPI(right_resized_, right_mat);
        
        // Validate downloaded images
        if (front_mat.empty() || rear_mat.empty() || left_mat.empty() || right_mat.empty()) {
            RCLCPP_ERROR(this->get_logger(), "One or more resized images are empty");
            return;
        }
        
        if (first_call) {
            RCLCPP_INFO(this->get_logger(), "Downloaded image sizes - Front: %dx%d, Rear: %dx%d, Left: %dx%d, Right: %dx%d",
                        front_mat.cols, front_mat.rows, rear_mat.cols, rear_mat.rows,
                        left_mat.cols, left_mat.rows, right_mat.cols, right_mat.rows);
            first_call = false;
        }
        
        // Create composite mosaic view
        cv::Mat composite(output_height_, output_width_, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // Resize images to fit quarter size if needed and arrange cameras in mosaic view layout
        cv::Mat left_resized, front_resized, rear_resized, right_resized;
        
        // Resize each image to quarter size
        cv::resize(left_mat, left_resized, cv::Size(quarter_width, quarter_height));
        cv::resize(front_mat, front_resized, cv::Size(quarter_width, quarter_height));
        cv::resize(rear_mat, rear_resized, cv::Size(quarter_width, quarter_height));
        cv::resize(right_mat, right_resized, cv::Size(quarter_width, quarter_height));
        
        // Arrange cameras in mosaic view layout
        // Top-left: Left camera
        left_resized.copyTo(composite(cv::Rect(0, 0, quarter_width, quarter_height)));
        
        // Top-right: Front camera
        front_resized.copyTo(composite(cv::Rect(quarter_width, 0, quarter_width, quarter_height)));
        
        // Bottom-left: Rear camera
        rear_resized.copyTo(composite(cv::Rect(0, quarter_height, quarter_width, quarter_height)));
        
        // Bottom-right: Right camera
        right_resized.copyTo(composite(cv::Rect(quarter_width, quarter_height, quarter_width, quarter_height)));
        
        // Upload composite back to VPI
        uploadToVPI(composite, output_vpi_);
    }
    
    void CHECK_STATUS(VPIStatus status)
    {
        if (status != VPI_SUCCESS) {
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];
            vpiGetLastStatusMessage(buffer, sizeof(buffer));
            RCLCPP_ERROR(this->get_logger(), "VPI Error: %s", buffer);
            throw std::runtime_error(buffer);
        }
    }
    
    // ROS2 members
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
    
    // VPI members
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
    
    // Parameters
    int output_width_;
    int output_height_;
    int camera_width_;
    int camera_height_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MosaicViewNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 