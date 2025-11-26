#include "geomagic_interface/geomagic_processor.hpp"

namespace geomagic_interface
{

GeomagicProcessor::GeomagicProcessor() : Node("geomagic_processor")
{
    // Declarar parámetros
    this->declare_parameter<std::string>("input_pose_topic", "/phantom/pose");
    this->declare_parameter<std::string>("input_button_topic", "/phantom/button");
    this->declare_parameter<double>("position_scale", 2.5);
    this->declare_parameter<double>("orientation_scale", 0.5);

    // Obtener parámetros
    this->get_parameter("input_pose_topic", input_pose_topic_);
    this->get_parameter("input_button_topic", input_button_topic_);
    this->get_parameter("position_scale", position_scale_);
    this->get_parameter("orientation_scale", orientation_scale_);

    // Suscriptores
    raw_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        input_pose_topic_, 10, 
        std::bind(&GeomagicProcessor::raw_pose_callback, this, std::placeholders::_1));
    
    button_sub_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
        input_button_topic_, 10,
        std::bind(&GeomagicProcessor::button_callback, this, std::placeholders::_1));

    // Publicadores
    processed_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/geomagic/processed_pose", 10);
    calibration_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/geomagic/calibration_status", 10);

    RCLCPP_INFO(this->get_logger(), "Procesador Geomagic iniciado");
    RCLCPP_INFO(this->get_logger(), "Entrada pose: %s", input_pose_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Entrada botones: %s", input_button_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Escala posición: %.2f, orientación: %.2f", 
                position_scale_, orientation_scale_);
}

void GeomagicProcessor::raw_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Actualizar estado háptico
    haptic_state_.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    haptic_state_.orientation.w() = msg->pose.orientation.w;
    haptic_state_.orientation.x() = msg->pose.orientation.x;
    haptic_state_.orientation.y() = msg->pose.orientation.y;
    haptic_state_.orientation.z() = msg->pose.orientation.z;

    // Si está calibrado, procesar y publicar
    if (calibrated_) {
        publish_processed_pose();
    }
}

void GeomagicProcessor::button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg)
{
    if (msg->grey_button == 1 && !calibrated_) {
        calibrate_device();
    }
    
    // Publicar eventos de botones para otros nodos
    if (msg->grey_button == 1) {
        RCLCPP_INFO(this->get_logger(), "Botón gris presionado");
    }
    if (msg->white_button == 1) {
        RCLCPP_INFO(this->get_logger(), "Botón blanco presionado");
    }
}

void GeomagicProcessor::publish_processed_pose()
{
    auto processed_msg = geometry_msgs::msg::PoseStamped();
    processed_msg.header.stamp = this->get_clock()->now();
    processed_msg.header.frame_id = "geomagic_processed";
    
    // Aplicar transformación y escalado de posición
    Eigen::Vector3d relative_position = haptic_state_.position - haptic_state_.position_initial;
    Eigen::Vector3d scaled_position = relative_position * position_scale_;
    
    processed_msg.pose.position.x = scaled_position.x();
    processed_msg.pose.position.y = scaled_position.y();
    processed_msg.pose.position.z = scaled_position.z();

    // Procesar orientación relativa
    Eigen::Quaterniond relative_orientation = haptic_state_.orientation * haptic_state_.orientation_initial.inverse();
    Eigen::AngleAxisd angle_axis(relative_orientation);
    Eigen::Quaterniond scaled_orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(orientation_scale_ * angle_axis.angle(), angle_axis.axis()));

    processed_msg.pose.orientation.w = scaled_orientation.w();
    processed_msg.pose.orientation.x = scaled_orientation.x();
    processed_msg.pose.orientation.y = scaled_orientation.y();
    processed_msg.pose.orientation.z = scaled_orientation.z();

    processed_pose_pub_->publish(processed_msg);
}

void GeomagicProcessor::calibrate_device()
{
    RCLCPP_INFO(this->get_logger(), "Calibrando dispositivo háptico...");
    
    haptic_state_.position_initial = haptic_state_.position;
    haptic_state_.orientation_initial = haptic_state_.orientation;
    calibrated_ = true;

    // Publicar estado de calibración
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = true;
    calibration_status_pub_->publish(status_msg);
    
    RCLCPP_INFO(this->get_logger(), "Calibración completada.");
    RCLCPP_INFO(this->get_logger(), "Posición inicial: [%.3f, %.3f, %.3f]",
                haptic_state_.position_initial.x(), 
                haptic_state_.position_initial.y(), 
                haptic_state_.position_initial.z());
}

} // namespace geomagic_interface

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<geomagic_interface::GeomagicProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}