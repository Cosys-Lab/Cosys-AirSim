#include <rclcpp/rclcpp.hpp>
#include "airsim_ros_wrapper.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("airsim_node", node_options);
    std::shared_ptr<rclcpp::Node> nh_img = nh->create_sub_node("img");
    std::shared_ptr<rclcpp::Node> nh_lidar = nh->create_sub_node("lidar");
    std::shared_ptr<rclcpp::Node> nh_gpulidar = nh->create_sub_node("gpulidar");
    std::shared_ptr<rclcpp::Node> nh_echo = nh->create_sub_node("echo");
    std::string host_ip;
    uint16_t host_port = 41451;
    bool enable_api_control = false;
    nh->get_parameter("host_ip", host_ip);
    nh->get_parameter("host_port", host_port);
    nh->get_parameter("enable_api_control", enable_api_control);
    auto callbackGroup = nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    AirsimROSWrapper airsim_ros_wrapper(nh, nh_img, nh_lidar, nh_gpulidar, nh_echo, host_ip, callbackGroup, enable_api_control, host_port);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();

    return 0;
}