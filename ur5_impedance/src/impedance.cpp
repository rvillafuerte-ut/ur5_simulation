#include "ur5_impedance/impedance.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <iostream>

UR5Impedance::UR5Impedance(const std::string& urdf_path) {
    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);    
    tool_frame_id_ = model_->getFrameId("tool0");
     
    if (!model_->existFrame("tool0")) {
        throw std::runtime_error("El frame 'tool0' no existe en el modelo URDF.");
    }

    J_previous_ = Eigen::MatrixXd::Zero(7, model_->nv);
    std::cout << "Modelo de impedancia cargado correctamente." << std::endl;
}

Eigen::MatrixXd UR5Impedance::computeFullJacobian(const Eigen::VectorXd& q) {
    Eigen::MatrixXd J_full(7, model_->nv);
    
    // Jacobiano estándar de 6D
    pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_full.topRows<6>());

    // Derivada del cuaternión (aproximación)
    // d(quat)/dt = 0.5 * quat * omega
    // Esto es complejo. Por simplicidad, la derivada del cuaternión en el jacobiano se puede aproximar
    // o calcular numéricamente, pero para el control es más robusto usar el error en el espacio tangente (log6).
    // Aquí mantenemos la estructura de tu código original, pero esto es un punto a mejorar.
    // Por ahora, la fila del cuaternión se deja en cero o se aproxima.
    // La forma más simple es usar el jacobiano geométrico de 6D y mapear el error de cuaternión a velocidad angular.
    
    // Para replicar tu código, usaremos el jacobiano de 6D y manejaremos el error de 7D en la ley de control.
    Eigen::MatrixXd J_geom(6, model_->nv);
    pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_geom);
    
    J_full.setZero();
    J_full.topRows(3) = J_geom.topRows(3); // Parte de posición
    // La parte de orientación del jacobiano de 7D es más compleja.
    // Por ahora, usaremos una simplificación.
    J_full.bottomRows(4).setZero(); // Placeholder

    return J_geom; // Devolvemos el jacobiano geométrico de 6D, que es más estándar.
}
Eigen::MatrixXd UR5Impedance::computeFullJacobianQuaternion(const Eigen::VectorXd& q, double delta = 1e-8)
{
    int nq = q.size();
    Eigen::MatrixXd J_full(7, nq); // 3 pos + 4 quat

    // Estado nominal
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    Eigen::Vector3d pos0 = data_->oMf[tool_frame_id_].translation();
    Eigen::Quaterniond quat0(data_->oMf[tool_frame_id_].rotation());

    for (int i = 0; i < nq; ++i) {
        Eigen::VectorXd q_perturbed = q;
        q_perturbed[i] += delta;

        pinocchio::forwardKinematics(*model_, *data_, q_perturbed);
        pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
        Eigen::Vector3d pos1 = data_->oMf[tool_frame_id_].translation();
        Eigen::Quaterniond quat1(data_->oMf[tool_frame_id_].rotation());

        // Diferencia numérica para posición
        Eigen::Vector3d dpos = (pos1 - pos0) / delta;
        // Diferencia numérica para cuaternión (Eigen almacena como x, y, z, w)
        Eigen::Vector4d dquat = (quat1.coeffs() - quat0.coeffs()) / delta;

        // Guardar en la columna i
        J_full.block<3,1>(0, i) = dpos;
        J_full.block<4,1>(3, i) = dquat;
    }
    return J_full;
}

Eigen::VectorXd UR5Impedance::calculateControlCommand(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Quaterniond& desired_orient,
    const Eigen::Vector3d& desired_vel,
    const Eigen::Vector3d& desired_acc,
    const Eigen::Matrix<double, 7, 1>& Kp_task_diag,
    const Eigen::Matrix<double, 7, 1>& Kd_task_diag,
    double dt)
{
    // 1. Cinemática y Jacobiano
    pinocchio::forwardKinematics(*model_, *data_, q, dq);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    
    Eigen::MatrixXd J(6, model_->nv);
    Eigen::MatrixXd J = computeFullJacobianQuaternion(q, 1e-8);
    Eigen::MatrixXd dJ = (J - J_anterior) / dt; //7x6
    Eigen::VectorXd dx_current_cartesian = J * dq; // (7x6) * (6x1) = 7x1

    Eigen::Vector4d vel_ori_d = Eigen::Vector4d::Zero();
    Eigen::Vector4d acc_ori_d = Eigen::Vector4d::Zero();
    Eigen::VectorXd vel_des(7), dvel_des(7);
    vel_des << v_des, vel_ori_d;
    dvel_des << a_des, acc_ori_d;

    Eigen::Matrix<double,7,7> Kp_task, Kd_task;
    Kp_task.setZero();
    Kd_task.setZero();
    Kp_task.diagonal() = Kp_task_diag;
    Kd_task.diagonal() = Kd_task_diag;

    // 2. Calcular errores
    const auto& current_pose = data_->oMf[tool_frame_id_];
    Eigen::Vector3d current_pos = current_pose.translation();
    Eigen::Quaterniond current_orient(current_pose.rotation());
    current_orient.normalize();

    // Error de posición
    Eigen::Vector3d error_pos = current_pos - desired_pos;

    // Error de orientación (diferencia de cuaterniones)
    Eigen::Vector4d error_quat = current_orient.coeffs() - desired_orient.coeffs();

    Eigen::Matrix<double, 7, 1> error_pose;
    error_pose << error_pos, error_quat;

    // Error de velocidad
    Eigen::VectorXd current_vel_6D = J * dq;
    Eigen::VectorXd desired_vel_7D(7);
    desired_vel_7D << desired_vel, Eigen::Vector4d::Zero();
    
    Eigen::VectorXd current_vel_7D(7);
    current_vel_7D << current_vel_6D.head(3), Eigen::Vector4d::Zero(); // Simplificación para la velocidad de orientación
    
    Eigen::VectorXd error_vel = desired_vel_7D - current_vel_7D;

    // 3. Dinámica del robot
    pinocchio::computeJointJacobians(*model_, *data_, q);
    pinocchio::crba(*model_, *data_, q); // Calcula Matriz de Inercia M
    pinocchio::computeCoriolisMatrix(*model_, *data_, q, dq);
    pinocchio::computeGeneralizedGravity(*model_, *data_, q);
    
    Eigen::MatrixXd M = data_->M;
    Eigen::VectorXd nle = data_->nle; // Coriolis + Gravedad

    // 4. Ley de control de impedancia
    // Replicando la fórmula: tau = M*J_pinv * (dvel_des - Kp*err_p - Kd*err_v - J*dq) + nle
    // Nota: La fórmula original tiene términos que se cancelan o son redundantes.
    // Una formulación más estándar es: tau = J^T * F + nle, donde F es la fuerza cartesiana deseada.
    
    Eigen::VectorXd desired_acc_7D(7);
    desired_acc_7D << desired_acc, Eigen::Vector4d::Zero();

    // Fuerza cartesiana deseada (simplificada para 7D)
    Eigen::Matrix<double, 7, 1> F_cartesian = desired_acc_7D - Kp_task_diag.asDiagonal() * error_pose - Kd_task_diag.asDiagonal() * error_vel;

    // Mapeo de fuerza a torque articular (usando el Jacobiano de 6D)
    Eigen::VectorXd tau = J.transpose() * F_cartesian.head(6) + nle;

    // 5. Integración para obtener la siguiente posición (esto debería hacerlo el controlador del robot, no la ley de control)
    // La ley de control debe devolver un TORQUE (tau) o una ACELERACIÓN (qdd).
    // Devolver la siguiente posición (q_solution) mezcla la ley de control con la simulación.
    // Es mejor devolver el torque y dejar que el nodo principal lo maneje.
    // Sin embargo, para replicar tu código, calcularemos q_solution.

    Eigen::MatrixXd M_inv = M.inverse();
    Eigen::VectorXd qdd = M_inv * (tau - nle); // Aceleración resultante
    
    Eigen::VectorXd qd_next = dq + qdd * dt;
    Eigen::VectorXd q_next = q + qd_next * dt;

    return q_next;
}