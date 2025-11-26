#include "ur5_sliding/sliding.hpp"

UR5Sliding::UR5Sliding(const std::string& urdf_path) {
    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);
    
    // Buscar el frame del efector final
    tool_frame_id_ = model_->getFrameId("tool0");
    
    if (!model_->existFrame("tool0")) {
        throw std::runtime_error("El frame 'tool0' no existe en el modelo URDF.");
    }

    // Inicializar el Jacobiano previo
    J_previous_ = Eigen::MatrixXd::Zero(6, model_->nv);
    
    std::cout << "Modelo de control deslizante cargado correctamente." << std::endl;
}

Eigen::VectorXd UR5Sliding::calculateControlCommand(
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
    double alpha,
    double damping_factor,
    double dt)
{
    // 1. Calcular todos los términos dinámicos (M, C, G)
    pinocchio::computeAllTerms(*model_, *data_, q, dq);
    
    // 2. Cinemática directa y actualizar frames
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacements(*model_, *data_);
    
    // 3. Obtener pose actual del efector final
    const pinocchio::SE3& current_pose = data_->oMf[tool_frame_id_];
    
    // 4. Calcular Jacobiano y su derivada
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, 
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    
    Eigen::MatrixXd J_dot = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::getFrameJacobianTimeVariation(*model_, *data_, tool_frame_id_,
                                            pinocchio::LOCAL_WORLD_ALIGNED, J_dot);
    
    // 5. Construir pose deseada
    pinocchio::SE3 pose_desired(desired_orient, desired_pos);
    
    // 6. Calcular error de posición
    Eigen::Vector3d pos_error = current_pose.translation() - pose_desired.translation();
    
    // 7. Calcular error de orientación usando log3
    Eigen::Matrix3d R_error = current_pose.rotation() * pose_desired.rotation().transpose();
    Eigen::Vector3d ori_error = pinocchio::log3(R_error);
    
    // 8. Vector de error completo [6x1]
    Eigen::VectorXd e(6);
    e << pos_error, ori_error;
    
    // 9. Velocidad cartesiana actual
    Eigen::VectorXd v_cartesian = J * dq;
    
    // 10. Velocidad cartesiana deseada
    Eigen::VectorXd v_desired(6);
    v_desired << desired_vel, desired_vel_ori;
    
    // 11. Error de velocidad
    Eigen::VectorXd e_v = v_cartesian - v_desired;
    
    // 12. Superficie deslizante: s = e_v + lambda .* e
    Eigen::VectorXd s = e_v + lambda.cwiseProduct(e);
    
    // 13. Función de signo suavizada con tanh
    Eigen::VectorXd s_sign = (alpha * s.array()).tanh().matrix();
    
    // 14. Aceleración cartesiana deseada
    Eigen::VectorXd a_desired(6);
    a_desired << desired_acc, desired_acc_ori;
    
    // 15. Ley de control SMC en el espacio cartesiano
    // a_cartesian_desired = a_desired - k2 .* tanh(alpha*s) - k .* s - lambda .* e_v
    Eigen::VectorXd a_cartesian_desired = a_desired 
                                        - k2.cwiseProduct(s_sign) 
                                        - k.cwiseProduct(s) 
                                        - lambda.cwiseProduct(e_v);
    
    // 16. Pseudoinversa amortiguada del Jacobiano
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd JJT_damped = J * J.transpose() + damping_factor * damping_factor * I;
    Eigen::MatrixXd J_pinv = J.transpose() * JJT_damped.inverse();
    
    // 17. Aceleración articular deseada
    // q_ddot_desired = J_pinv * (a_cartesian_desired - J_dot * dq)
    Eigen::VectorXd q_ddot_desired = J_pinv * (a_cartesian_desired - J_dot * dq);
    
    // 18. Obtener términos dinámicos
    Eigen::MatrixXd M = data_->M;           // Matriz de inercia
    Eigen::VectorXd Cq = data_->C * dq;     // Términos de Coriolis
    Eigen::VectorXd G = data_->g;           // Términos de gravedad
    
    // 19. Calcular par de control usando el modelo dinámico
    // tau = M * q_ddot_desired + C*dq + G
    Eigen::VectorXd tau = M * q_ddot_desired + Cq + G;

    //calculo de q_desired
    Eigen::VectorXd q_desired = q + dt * dq + 0.5 * dt * dt * q_ddot_desired;
    
    // Actualizar Jacobiano previo para la próxima iteración
    J_previous_ = J;



    return q_desired;
}

Eigen::Matrix<double, 6, 1> UR5Sliding::computePoseError(
    const Eigen::VectorXd& q,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Matrix3d& desired_orient)
{
    // Cinemática directa
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    
    const pinocchio::SE3& current_pose = data_->oMf[tool_frame_id_];
    
    // Error de posición
    Eigen::Vector3d pos_error = current_pose.translation() - desired_pos;
    
    // Error de orientación
    Eigen::Matrix3d R_error = current_pose.rotation() * desired_orient.transpose();
    Eigen::Vector3d ori_error = pinocchio::log3(R_error);
    
    Eigen::Matrix<double, 6, 1> error;
    error << pos_error, ori_error;
    
    return error;
}

pinocchio::SE3 UR5Sliding::getCurrentPose(const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    return data_->oMf[tool_frame_id_];
}

Eigen::MatrixXd UR5Sliding::computeJacobian(const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_,
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    return J;
}

Eigen::MatrixXd UR5Sliding::computeJacobianDerivative(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq)
{
    pinocchio::forwardKinematics(*model_, *data_, q, dq);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    
    Eigen::MatrixXd J_dot = Eigen::MatrixXd::Zero(6, model_->nv);
    pinocchio::getFrameJacobianTimeVariation(*model_, *data_, tool_frame_id_,
                                            pinocchio::LOCAL_WORLD_ALIGNED, J_dot);
    return J_dot;
}
