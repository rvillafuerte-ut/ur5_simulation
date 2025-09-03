#ifndef UR5_IMPEDANCE_HPP
#define UR5_IMPEDANCE_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>
#include <string>
#include <memory>

class UR5Impedance {
public:
    // El constructor carga el modelo del robot.
    explicit UR5Impedance(const std::string& urdf_path);
    
    // El método principal que calcula la siguiente posición articular.
    Eigen::VectorXd calculateControlCommand(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& dq,
        const Eigen::Vector3d& desired_pos,
        const Eigen::Quaterniond& desired_orient,
        const Eigen::Vector3d& desired_vel,
        const Eigen::Vector3d& desired_acc,
        const Eigen::Matrix<double, 7, 1>& Kp_task,
        const Eigen::Matrix<double, 7, 1>& Kd_task,
        double dt
    );

private:
    // Miembros privados para el modelo y los datos de Pinocchio.
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex tool_frame_id_;

    // Jacobiano de la iteración anterior para calcular la derivada.
    Eigen::MatrixXd J_previous_;

    // Métodos auxiliares privados.
    Eigen::MatrixXd computeFullJacobian(const Eigen::VectorXd& q);
    Eigen::MatrixXd computeFullJacobianQuaternion(
        const pinocchio::Model& model,
        pinocchio::Data& data,
        const pinocchio::FrameIndex& tool_frame_id,
        const Eigen::VectorXd& q,
        double delta = 1e-8)
};

#endif // UR5_IMPEDANCE_HPP