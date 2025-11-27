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
  // ------------------------------------------------------------------
  // Identificación y namespaces
  // ------------------------------------------------------------------
  std::string control_topic = "/joint_trajectory_controller/joint_trajectory"; // Tópico de control articular
  std::string operation_mode = "teleop";   // "teleop" o "trajectory"
  std::string ur_model = "ur5";            // Modelo: ur5 / ur5e
  std::string nmspace = "";                // Namespace (prefijo de joints y tópicos)
  std::string urdf_path;                    // Ruta absoluta al URDF (se completa en runtime si queda vacío)
  std::string controller="QP";
  bool use_geomagic = false;                // Modo teleoperado háptico

  // ------------------------------------------------------------------
  // Frecuencia y lazo de control
  // ------------------------------------------------------------------
  double control_frequency = 100.0;         // Hz (referencial)
  double control_loop_time = 0.01;          // Segundos para time_from_start de cada punto

  // ------------------------------------------------------------------
  // Movimiento articular inicial (similar a nodo ur5_pos)
  // ------------------------------------------------------------------
  bool use_ur5_pos_init = true;             // Activar secuencia inicial hacia q_target
  std::vector<double> q_target {1.57, -1.9, 1.7, -1.9, -1.7, 0.0}; // Objetivo articular inicial
  double q_target_time = 2.0;               // Tiempo para alcanzar objetivo inicial

  // ------------------------------------------------------------------
  // Trayectoria automática (cuando use_geomagic = false)
  // Parámetros leídos de ROS params: traj_A, traj_wn, traj_c0, traj_mode
  // ------------------------------------------------------------------
  Eigen::Vector3d traj_A {0.05, 0.05, 0.05}; // Amplitudes en metros
  double traj_wn = 3.14159;                  // Frecuencia natural
  double traj_c0 = 1.0;                      // Factor de decaimiento
  int traj_mode = 1;                         // Modo (1 sinusoidal, 2 exponencial, etc.)
  std::string trayectoria = "cicloide creciente"; // Descripción textual (legacy)

  // ------------------------------------------------------------------
  // Waypoints de trayectoria personalizada (no implementado aún)
  // ------------------------------------------------------------------
  std::vector<double> trajectory_waypoints;  // Reservado para futuras trayectorias definidas por puntos
  double trajectory_duration = 5.0;          // Duración total de trayectoria de waypoints

  // ------------------------------------------------------------------
  // CSV Logging
  // ------------------------------------------------------------------
  bool csv_enabled = false;                 // Activar log CSV
  std::string csv_path = "";               // Directorio destino (se completa si vacío)
  std::string csv_prefix = "ur5_log";       // Prefijo de nombre de archivo
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
