#include "econ_mosaic_vpi/mosaic_view_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MosaicViewNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 