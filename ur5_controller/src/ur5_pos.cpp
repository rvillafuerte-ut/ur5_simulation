#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <string>
#include <memory>

class UR5_positioner : public rclcpp::Node
{
public:
  UR5_positioner() : Node("ur5_positioner")
  {
    // 1. Declarar todos los parámetros con sus tipos y valores por defecto
    this->declare_parameter<std::string>("ur_name", "ur5"); // Nombre del robot
    this->declare_parameter<std::string>("control_topic", "/scaled_joint_trajectory_controller/joint_trajectory");
    this->declare_parameter<std::string>("ur", "ur5");
    this->declare_parameter<double>("ur5_time", 2.0); // Tiempo para alcanzar la posición en segundos
    this->declare_parameter<std::vector<double>>("q", {0.0, -1.57, 1.57, -1.57, -1.57, 0.0}); // Posición articular por defecto

    // 2. Obtener los valores de los parámetros
    
    std::string topic_name = this->get_parameter("control_topic").as_string();
    std::string ur_model = this->get_parameter("ur").as_string();
    double ur5_time = this->get_parameter("ur5_time").as_double();
    ur_name = this->get_parameter("ur_name").as_string()+"_";
    std::vector<double> q_solution = this->get_parameter("q").as_double_array();
    nmspace = "/" + ur_model;
    RCLCPP_INFO(this->get_logger(), "Configurando posicionador para el robot: '%s'", ur_model.c_str());
    RCLCPP_INFO(this->get_logger(), "Publicando en el tópico de control: '%s'", (nmspace + topic_name).c_str());
    if (q_solution.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "El parámetro 'q' debe contener 6 valores articulares. Se recibieron %zu.", q_solution.size());
        return;
    }

    // 3. Crear el publicador
    
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(nmspace + topic_name, 10);

    // Esperar un momento para asegurar que el publicador esté listo
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 4. Publicar el mensaje una sola vez
    publish_position(q_solution, ur5_time);

    RCLCPP_INFO(this->get_logger(), "Posición publicada. El nodo se cerrará en breve.");
    // Opcional: hacer que el nodo se apague después de publicar
    // rclcpp::shutdown(); 
  }
 
private:
    std::string ur_name;
    std::string nmspace;

    void publish_position(const std::vector<double>& positions, double time_to_reach) {
        auto trajectory_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
        trajectory_msg->joint_names = {ur_name + "shoulder_pan_joint", ur_name + "shoulder_lift_joint", ur_name + "elbow_joint",
                                       ur_name + "wrist_1_joint", ur_name + "wrist_2_joint", ur_name + "wrist_3_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start = rclcpp::Duration::from_seconds(time_to_reach);

        trajectory_msg->points.push_back(point);
        joint_trajectory_pub_->publish(std::move(trajectory_msg));
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR5_positioner>();
  // Como el nodo publica una vez y termina su trabajo, no es estrictamente necesario un spin.
  // Pero lo mantenemos para que el nodo siga vivo y los mensajes de INFO se vean.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/// Usar de esta forma, desde terminal:
//ros2 run ur5_controller ur5_pos --ros-args -p q:="[1.0, -1.0, 1.0, -1.57, -1.57, 0.0]" -p ur5_time:=4.0 -p control_topic:="/scaled_joint_trajectory_controller/joint_trajectory"
//ros2 run ur5_controller ur5_pos --ros-args -p q:="[1.0, -1.0, 1.0, -1.57, -1.57, 0.0]" -p ur5_time:=4.0 -p control_topic:="/joint_trajectory_controller/joint_trajectory"