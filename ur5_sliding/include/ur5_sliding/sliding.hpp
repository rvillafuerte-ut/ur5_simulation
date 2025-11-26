#ifndef UR5_SLIDING_HPP
#define UR5_SLIDING_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>

#define PI 3.14159265358979323846

/**
 * @brief Clase para control por modo deslizante (Sliding Mode Control) del UR5
 * 
 * Esta clase implementa un controlador SMC en el espacio de trabajo cartesiano
 * para seguimiento de trayectorias del robot UR5.
 */
class UR5Sliding {
public:
    /**
     * @brief Constructor que carga el modelo URDF del robot
     * @param urdf_path Ruta al archivo URDF del robot
     */
    UR5Sliding(const std::string& urdf_path);

    /**
     * @brief Calcula el comando de control usando modo deslizante
     * 
     * @param q Posiciones articulares actuales [6x1]
     * @param dq Velocidades articulares actuales [6x1]
     * @param desired_pos Posición cartesiana deseada [3x1]
     * @param desired_orient Orientación deseada (matriz de rotación 3x3)
     * @param desired_vel Velocidad cartesiana deseada (lineal) [3x1]
     * @param desired_vel_ori Velocidad angular deseada [3x1]
     * @param desired_acc Aceleración cartesiana deseada (lineal) [3x1]
     * @param desired_acc_ori Aceleración angular deseada [3x1]
     * @param lambda Ganancias lambda para la superficie deslizante [6x1]
     * @param k Ganancias k para el término proporcional [6x1]
     * @param k2 Ganancias k2 para el término de signo [6x1]
     * @param alpha Parámetro de suavizado para la función tanh (default: 10.0)
     * @param damping_factor Factor de amortiguamiento para la pseudoinversa (default: 0.05)
     * @param dt Paso de tiempo para integración [s]
     * 
     * @return Par motor calculado (tau) [6x1]
     */
    Eigen::VectorXd calculateControlCommand(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& dq,
        const Eigen::Vector3d& desired_pos,
        const Eigen::Matrix3d& desired_orient,
        const Eigen::Vector3d& desired_vel,
        const Eigen::Vector3d& desired_vel_ori,
        const Eigen::Vector3d& desired_acc,
        const Eigen::Vector3d& desired_acc_ori,
        const Eigen::Matrix<double, 6, 1>& lambda,
        const Eigen::Matrix<double, 6, 1>& k,
        const Eigen::Matrix<double, 6, 1>& k2,
        double alpha = 10.0,
        double damping_factor = 0.05,
        double dt = 0.02);

    /**
     * @brief Calcula el error de pose actual respecto a la deseada
     * 
     * @param q Posiciones articulares actuales
     * @param desired_pos Posición deseada
     * @param desired_orient Orientación deseada (matriz de rotación)
     * @return Vector de error [6x1]: [error_pos, error_ori]
     */
    Eigen::Matrix<double, 6, 1> computePoseError(
        const Eigen::VectorXd& q,
        const Eigen::Vector3d& desired_pos,
        const Eigen::Matrix3d& desired_orient);

    /**
     * @brief Obtiene la pose actual del efector final
     * 
     * @param q Posiciones articulares actuales
     * @return Transformación SE3 de la pose actual
     */
    pinocchio::SE3 getCurrentPose(const Eigen::VectorXd& q);

    /**
     * @brief Calcula el Jacobiano del efector final
     * 
     * @param q Posiciones articulares actuales
     * @return Jacobiano [6xn]
     */
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q);

    /**
     * @brief Calcula la derivada temporal del Jacobiano
     * 
     * @param q Posiciones articulares actuales
     * @param dq Velocidades articulares actuales
     * @return Derivada del Jacobiano [6xn]
     */
    Eigen::MatrixXd computeJacobianDerivative(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& dq);

private:
    std::unique_ptr<pinocchio::Model> model_;  ///< Modelo del robot
    std::unique_ptr<pinocchio::Data> data_;    ///< Datos de cálculo de Pinocchio
    pinocchio::FrameIndex tool_frame_id_;      ///< ID del frame del efector final
    
    Eigen::MatrixXd J_previous_;  ///< Jacobiano previo para calcular derivada
};

#endif // UR5_SLIDING_HPP