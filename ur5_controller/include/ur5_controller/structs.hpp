#ifndef UR5_CONTROLLER__STRUCTS_HPP_
#define UR5_CONTROLLER__STRUCTS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <memory>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace ur5_controller
{

// Estado del robot (articular)
struct RobotState
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);      // Posiciones articulares actuales
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(6);     // Velocidades articulares actuales
  Eigen::VectorXd q_init = Eigen::VectorXd::Zero(6); // Posiciones articulares iniciales
  Eigen::VectorXd q_solution = Eigen::VectorXd::Zero(6); // Solución del controlador
};

// Estado Cartesiano del efector final
struct CartesianState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();      // Posición cartesiana actual
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // Orientación cartesiana actual
  Eigen::Vector3d position_desired = Eigen::Vector3d::Zero(); // Posición cartesiana deseada
  Eigen::Quaterniond orientation_desired = Eigen::Quaterniond::Identity(); // Orientación cartesiana deseada
  Eigen::Vector3d position_initial = Eigen::Vector3d::Zero(); // Posición cartesiana inicial
  Eigen::Quaterniond orientation_initial = Eigen::Quaterniond::Identity(); // Orientación inicial
};

// Estado del dispositivo háptico (Geomagic)
struct HapticState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();           // Posición actual
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); // Orientación actual
  Eigen::Vector3d position_initial = Eigen::Vector3d::Zero();      // Posición inicial capturada
  Eigen::Quaterniond orientation_initial = Eigen::Quaterniond::Identity(); // Orientación inicial capturada
};

// Configuración y parámetros del nodo
struct NodeConfig
{
  std::string control_topic;
  std::string operation_mode = "teleop";  // "teleop" o "trajectory"
  std::string ur_model = "ur5";
  std::string nmspace = "";
  std::string urdf_path;
  bool use_geomagic = false;              // Para compatibilidad con controller_backup
  std::vector<double> trajectory_waypoints;
  double trajectory_duration = 5.0;
  double control_frequency = 100.0;
  double control_loop_time = 0.01; // segundos
  std::string trayectoria = "cicloide creciente";
  bool csv_enabled = false;
  std::string csv_path = "robot_log.csv";
  std::string csv_prefix = "ur5_log";
};

// Recursos de Pinocchio
struct PinocchioResources
{
    std::unique_ptr<pinocchio::Model> model;
    std::unique_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex tool_frame_id;
};

} // namespace ur5_controller

#endif // UR5_CONTROLLER__STRUCTS_HPP_
