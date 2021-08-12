#include <rclcpp/rclcpp.hpp>
#include <depthai_ros_driver/depthai_base_ros2.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rr::DepthAIBaseRos2>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
