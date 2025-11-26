#include <ur5_controller/structs.hpp>

// ROS 2 Core
#include <rclcpp/rclcpp.hpp>

// Mensajes
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

// Servicios
#include <ur5_kinematics_server/srv/inverse_kinematics.hpp>

// Eigen para matemáticas
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Utilidades de ROS 2
#include <ament_index_cpp/get_package_share_directory.hpp>

// Standard C++
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

// Usar el namespace para las structs
using namespace ur5_controller;

class TrajectoryGenerator {
public:
    // Estructura para contener los resultados
    struct State {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };

    // Calcula el estado completo de la trayectoria en un tiempo dado
    static State calculate(
        const Eigen::Vector3d& x_init, 
        const Eigen::Vector3d& A, 
        double wn, 
        double c0, 
        double time_elapsed) 
    {
        State state;
        double t = time_elapsed;

        // --- Términos comunes (se calculan una sola vez) ---
        double exp_neg_c0_t = exp(-c0 * t);
        double sin_wn_t = sin(wn * t);
        double cos_wn_t = cos(wn * t);
        
        double amp_factor = 1.0 - exp_neg_c0_t;
        double d_amp_factor_dt = c0 * exp_neg_c0_t;
        double d2_amp_factor_dt2 = -c0 * c0 * exp_neg_c0_t;
        // ----------------------------------------------------

        // --- Cálculo de Posición ---
        state.position.x() = x_init.x() + A.x() * amp_factor * sin_wn_t;
        state.position.y() = x_init.y() + A.y() * amp_factor * cos_wn_t;
        state.position.z() = x_init.z() + A.z() * amp_factor * sin_wn_t;

        // --- Cálculo de Velocidad ---
        state.velocity.x() = A.x() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);
        state.velocity.y() = A.y() * (d_amp_factor_dt * cos_wn_t - amp_factor * wn * sin_wn_t);
        state.velocity.z() = A.z() * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);

        // --- Cálculo de Aceleración ---
        double term1_sin = d2_amp_factor_dt2 - amp_factor * wn * wn;
        double term2_cos = 2 * d_amp_factor_dt * wn;
        state.acceleration.x() = A.x() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
        state.acceleration.y() = A.y() * (term1_sin * cos_wn_t - term2_cos * sin_wn_t);
        state.acceleration.z() = A.z() * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);

        return state;
    }
};

std::string get_file_path(const std::string& package_name, const std::string& relative_path) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        return package_share_directory + "/" + relative_path;
    } catch (const std::exception& e) {
        throw std::runtime_error("No se pudo encontrar el paquete: " + package_name);
    }
}

class UR5ModularController : public rclcpp::Node
{
public:
    UR5ModularController() : Node("ur5_modular_controller")
    {
        // 1. Declarar parámetros
        this->declare_parameter<std::string>("control_topic", "/joint_trajectory_controller/joint_trajectory");
        this->declare_parameter<std::string>("operation_mode", "teleop"); // "teleop" o "trajectory"
        this->declare_parameter<std::string>("ur_model", "ur5");
        this->declare_parameter<std::vector<double>>("trajectory_waypoints", std::vector<double>{});
        this->declare_parameter<double>("trajectory_duration", 5.0);
        this->declare_parameter<double>("control_frequency", 100.0);

        // 2. Obtener parámetros
        this->get_parameter("control_topic", config_.control_topic);
        this->get_parameter("operation_mode", config_.operation_mode);
        this->get_parameter("ur_model", config_.ur_model);
        this->get_parameter("trajectory_waypoints", config_.trajectory_waypoints);
        this->get_parameter("trajectory_duration", config_.trajectory_duration);
        this->get_parameter("control_frequency", config_.control_frequency);

        // 3. Validar modo de operación
        if (config_.operation_mode != "teleop" && config_.operation_mode != "trajectory") {
            RCLCPP_ERROR(this->get_logger(), "Modo de operación no válido: %s. Use 'teleop' o 'trajectory'", config_.operation_mode.c_str());
            throw std::invalid_argument("Modo de operación no válido");
        }

        RCLCPP_INFO(this->get_logger(), "=== CONFIGURACIÓN DEL CONTROLADOR ===");
        RCLCPP_INFO(this->get_logger(), "Modo de operación: %s", config_.operation_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "Tópico de control: %s", config_.control_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Frecuencia de control: %.1f Hz", config_.control_frequency);

        // 4. Inicializar publicadores y suscriptores
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(config_.control_topic, 10);
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
            std::bind(&UR5ModularController::update_joint_positions, this, std::placeholders::_1));

        // 5. Inicializar cliente de cinemática inversa
        kinematics_client_ = this->create_client<ur5_kinematics_server::srv::InverseKinematics>("/ur5_kinematics_server/inverse_kinematics");

        // 6. Configurar según el modo de operación
        if (config_.operation_mode == "teleop") {
            setup_teleoperation();
        } else if (config_.operation_mode == "trajectory") {
            setup_trajectory_mode();
        }

        // 7. Timer de control principal
        auto control_period = std::chrono::duration<double>(1.0 / config_.control_frequency);
        timer_ = this->create_wall_timer(control_period, std::bind(&UR5ModularController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "=== CONTROLADOR MODULAR UR5 INICIADO ===");
    }

private:
    // ROS 2 objects
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr geomagic_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibration_status_sub_;
    rclcpp::Client<ur5_kinematics_server::srv::InverseKinematics>::SharedPtr kinematics_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Variables agrupadas en structs ---
    RobotState robot_state_;
    CartesianState cartesian_state_;
    HapticState haptic_state_;
    NodeConfig config_;
    // ------------------------------------

    // Estado del sistema
    bool system_initialized_ = false;
    bool geomagic_calibrated_ = false;
    
    // Variables para modo trayectoria
    std::chrono::steady_clock::time_point trajectory_start_time_;
    bool trajectory_started_ = false;

    void setup_teleoperation() {
        RCLCPP_INFO(this->get_logger(), "Configurando modo teleoperation...");
        
        // Suscribirse a la pose procesada del Geomagic
        geomagic_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/geomagic/processed_pose", 10,
            std::bind(&UR5ModularController::geomagic_pose_callback, this, std::placeholders::_1));

        // Suscribirse al estado de calibración
        calibration_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/geomagic/calibration_status", 10,
            std::bind(&UR5ModularController::calibration_status_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Teleoperation configurada. Esperando datos del Geomagic...");
    }

    void setup_trajectory_mode() {
        RCLCPP_INFO(this->get_logger(), "Configurando modo trajectory...");
        
        if (config_.trajectory_waypoints.empty()) {
            // Trayectoria por defecto - movimiento circular
            RCLCPP_INFO(this->get_logger(), "Usando trayectoria circular por defecto");
        } else {
            RCLCPP_INFO(this->get_logger(), "Usando trayectoria personalizada con %zu waypoints", 
                        config_.trajectory_waypoints.size() / 6);
        }
    }

    void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Mapeo de articulaciones según el controlador
        if (config_.control_topic == "/scaled_joint_trajectory_controller/joint_trajectory") {
            robot_state_.q << msg->position[5], msg->position[0], msg->position[1], 
                             msg->position[2], msg->position[3], msg->position[4];
            robot_state_.qd << msg->velocity[5], msg->velocity[0], msg->velocity[1], 
                              msg->velocity[2], msg->velocity[3], msg->velocity[4];
        } else {
            for (int i = 0; i < 6; ++i) {
                robot_state_.q[i] = msg->position[i];
                robot_state_.qd[i] = msg->velocity[i];
            }
        }

        // Inicialización del sistema en la primera recepción
        if (!system_initialized_) {
            initialize_system();
        }
    }

    void initialize_system() {
        RCLCPP_INFO(this->get_logger(), "Inicializando sistema...");
        
        robot_state_.q_init = robot_state_.q;
        
        // Obtener pose inicial usando el servicio de cinemática
        auto request = std::make_shared<ur5_kinematics_server::srv::InverseKinematics::Request>();
        // Para la pose inicial, solo necesitamos calcular la cinemática directa
        // Pero como el servicio es para IK, usaremos una posición temporal
        request->target_position.x = 0.0;
        request->target_position.y = 0.0;
        request->target_position.z = 0.0;
        request->target_orientation.w = 1.0;
        request->current_joint_positions.assign(robot_state_.q.data(), robot_state_.q.data() + 6);
        request->max_iterations = 100;
        request->tolerance = 0.1;

        // Por ahora, asumimos una pose inicial estándar
        cartesian_state_.position_initial << 0.4, 0.0, 0.4;
        cartesian_state_.orientation_initial = Eigen::Quaterniond::Identity();
        
        system_initialized_ = true;
        
        if (config_.operation_mode == "trajectory") {
            trajectory_start_time_ = std::chrono::steady_clock::now();
            trajectory_started_ = true;
        }
        
        RCLCPP_INFO(this->get_logger(), "Sistema inicializado. Control activo.");
        RCLCPP_INFO(this->get_logger(), "q_init: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                    robot_state_.q_init[0], robot_state_.q_init[1], robot_state_.q_init[2], 
                    robot_state_.q_init[3], robot_state_.q_init[4], robot_state_.q_init[5]);
    }

    void geomagic_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!geomagic_calibrated_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Esperando calibración del Geomagic...");
            return;
        }

        // Los datos ya vienen procesados del geomagic_interface
        haptic_state_.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        haptic_state_.orientation.w() = msg->pose.orientation.w;
        haptic_state_.orientation.x() = msg->pose.orientation.x;
        haptic_state_.orientation.y() = msg->pose.orientation.y;
        haptic_state_.orientation.z() = msg->pose.orientation.z;

        // Calcular pose deseada para el robot
        cartesian_state_.position_desired = cartesian_state_.position_initial + haptic_state_.position;
        cartesian_state_.orientation_desired = cartesian_state_.orientation_initial * haptic_state_.orientation;
    }

    void calibration_status_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        geomagic_calibrated_ = msg->data;
        if (geomagic_calibrated_) {
            RCLCPP_INFO(this->get_logger(), "Geomagic calibrado. Teleoperation activa.");
        }
    }

    void control_loop() {
        if (!system_initialized_) {
            return;
        }

        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_orientation;

        // Determinar posición objetivo según el modo
        if (config_.operation_mode == "teleop") {
            if (!geomagic_calibrated_) {
                return;
            }
            target_position = cartesian_state_.position_desired;
            target_orientation = cartesian_state_.orientation_desired;
        } else if (config_.operation_mode == "trajectory") {
            calculate_trajectory_target(target_position, target_orientation);
        }

        // Llamar al servicio de cinemática inversa
        call_inverse_kinematics_service(target_position, target_orientation);
    }

    void calculate_trajectory_target(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
        if (!trajectory_started_) {
            position = cartesian_state_.position_initial;
            orientation = cartesian_state_.orientation_initial;
            return;
        }

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - trajectory_start_time_);
        double t = elapsed.count();

        if (config_.trajectory_waypoints.empty()) {
            // Trayectoria circular por defecto
            double radius = 0.1;
            double frequency = 0.2; // Hz
            double omega = 2.0 * M_PI * frequency;
            
            position = cartesian_state_.position_initial;
            position.x() += radius * cos(omega * t);
            position.y() += radius * sin(omega * t);
            
            orientation = cartesian_state_.orientation_initial;
        } else {
            // Trayectoria personalizada (implementar interpolación entre waypoints)
            // Por simplicidad, mantener posición inicial por ahora
            position = cartesian_state_.position_initial;
            orientation = cartesian_state_.orientation_initial;
        }
    }

    void call_inverse_kinematics_service(const Eigen::Vector3d& target_position, const Eigen::Quaterniond& target_orientation) {
        if (!kinematics_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Servicio de cinemática no disponible");
            return;
        }

        auto request = std::make_shared<ur5_kinematics_server::srv::InverseKinematics::Request>();
        
        // Llenar la solicitud
        request->target_position.x = target_position.x();
        request->target_position.y = target_position.y();
        request->target_position.z = target_position.z();
        
        request->target_orientation.w = target_orientation.w();
        request->target_orientation.x = target_orientation.x();
        request->target_orientation.y = target_orientation.y();
        request->target_orientation.z = target_orientation.z();
        
        request->current_joint_positions.assign(robot_state_.q.data(), robot_state_.q.data() + 6);
        request->max_iterations = 600;
        request->tolerance = 0.1;

        // Llamada síncrona simplificada
        try {
            auto future_result = kinematics_client_->async_send_request(request);
            
            // Esperar la respuesta con timeout muy corto para no bloquear
            auto status = future_result.future.wait_for(std::chrono::milliseconds(10));
            
            if (status == std::future_status::ready) {
                auto response = future_result.future.get();
                handle_kinematics_response(response);
            } else {
                // Si no está listo, usar la solución anterior para continuar
                robot_state_.q_solution = robot_state_.q;
                publish_joint_trajectory();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en llamada IK: %s", e.what());
            // Usar posición actual en caso de error
            robot_state_.q_solution = robot_state_.q;
            publish_joint_trajectory();
        }
    }

    void handle_kinematics_response(const std::shared_ptr<ur5_kinematics_server::srv::InverseKinematics::Response> response) {        
        if (response->success) {
            // Actualizar solución de articulaciones
            for (int i = 0; i < 6; ++i) {
                robot_state_.q_solution[i] = response->joint_solution[i];
            }
            
            // Publicar trayectoria
            publish_joint_trajectory();
            
            RCLCPP_DEBUG(this->get_logger(), "IK exitoso: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                        robot_state_.q_solution[0], robot_state_.q_solution[1], robot_state_.q_solution[2], 
                        robot_state_.q_solution[3], robot_state_.q_solution[4], robot_state_.q_solution[5]);
        } else {
            RCLCPP_WARN(this->get_logger(), "IK falló: %s", response->message.c_str());
        }
    }

    void publish_joint_trajectory() {
        auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
        trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions.assign(robot_state_.q_solution.data(), robot_state_.q_solution.data() + 6);
        
        // Tiempo adaptativo basado en la distancia al objetivo
        double distance_to_target = (robot_state_.q - robot_state_.q_solution).norm();
        double adaptive_time = std::max(0.1, std::min(1.0, distance_to_target * 2.0));
        
        point.time_from_start = rclcpp::Duration::from_seconds(adaptive_time);
        trajectory_msg.points.push_back(point);
        
        joint_trajectory_pub_->publish(trajectory_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<UR5ModularController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error en el controlador: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}