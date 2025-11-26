#ifndef GEOMAGIC_INTERFACE__GEOMAGIC_PROCESSOR_HPP_
#define GEOMAGIC_INTERFACE__GEOMAGIC_PROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <omni_msgs/msg/omni_button_event.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace geomagic_interface
{

struct HapticState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position_initial = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_initial = Eigen::Quaterniond::Identity();
};

class GeomagicProcessor : public rclcpp::Node
{
public:
    GeomagicProcessor();

private:
    // ROS 2 objects
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pose_sub_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr processed_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_status_pub_;

    // Estado
    HapticState haptic_state_;
    bool calibrated_ = false;
    
    // Parámetros
    double position_scale_ = 2.5;
    double orientation_scale_ = 0.5;
    std::string input_pose_topic_;
    std::string input_button_topic_;

    // Callbacks
    void raw_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg);
    
    // Utilidades
    void publish_processed_pose();
    void calibrate_device();
};

} // namespace geomagic_interface

#endif // GEOMAGIC_INTERFACE__GEOMAGIC_PROCESSOR_HPP_