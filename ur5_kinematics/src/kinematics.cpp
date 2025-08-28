#include "ur5_kinematics/kinematics.hpp"
#include <iostream>
#include <OsqpEigen/OsqpEigen.h>

UR5Kinematics::UR5Kinematics(const std::string& urdf_path) {
    model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *model_);
    data_ = std::make_unique<pinocchio::Data>(*model_);
    tool_frame_id_ = model_->getFrameId("tool0");

    if (!model_->existFrame("tool0")) {
        throw std::runtime_error("El frame 'tool0' no existe en el modelo URDF.");
    }
    std::cout << "Modelo cinemático cargado correctamente desde " << urdf_path << std::endl;
}

pinocchio::SE3 UR5Kinematics::forwardKinematics(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
    return data_->oMf[tool_frame_id_];
}

Eigen::VectorXd UR5Kinematics::inverseKinematics(
    const Eigen::VectorXd& q_initial,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Quaterniond& desired_orient,
    int max_iterations,
    double alpha)
{
    Eigen::VectorXd q = q_initial;
    pinocchio::SE3 desired_pose(desired_orient.toRotationMatrix(), desired_pos);

    for (int i = 0; i < max_iterations; ++i) {
        pinocchio::forwardKinematics(*model_, *data_, q);
        pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);
        
        const pinocchio::SE3& current_pose = data_->oMf[tool_frame_id_];
        pinocchio::Data::Matrix6x J(6, model_->nv);
        J.setZero();
        pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

        Eigen::Matrix<double, 6, 1> error = pinocchio::log6(desired_pose.inverse() * current_pose).toVector();
        
        if (error.norm() < 1e-4) {
            // std::cout << "Convergencia alcanzada en " << i << " iteraciones." << std::endl;
            return q;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd J_pinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();
        
        q -= alpha * J_pinv * error;
    }
    // std::cerr << "Advertencia: La cinemática inversa no convergió." << std::endl;
    return q;
}

// --- NUEVAS FUNCIONES PARA IK CON SOLVER QP ---

Eigen::Matrix<double, 6, 1> UR5Kinematics::computePoseError(const pinocchio::SE3& desired_pose) {
    const pinocchio::SE3 current_pose = data_->oMf[tool_frame_id_];

    // Error de posición
    Eigen::Vector3d position_error = desired_pose.translation() - current_pose.translation();

    // Error de orientación usando cuaterniones
    Eigen::Quaterniond current_orientation = Eigen::Quaterniond(current_pose.rotation());
    Eigen::Quaterniond desired_orientation = Eigen::Quaterniond(desired_pose.rotation());

    Eigen::Quaterniond error_quat = desired_orientation * current_orientation.inverse();
    Eigen::Vector3d angular_error;

    // Convertir el cuaternión de error a una representación vectorial del error angular
    if (error_quat.w() < 0) {
        error_quat.w() = -error_quat.w(); // Asegurar la parte escalar positiva para la representación del error más corta
    }
    angular_error = error_quat.vec(); // El vector parte del cuaternión representa el eje de rotación escalado por sin(ángulo/2)

    Eigen::VectorXd error(6);
    error << position_error, angular_error;
    return error;
}

Eigen::VectorXd UR5Kinematics::solveQPIK(const Eigen::MatrixXd& J, const Eigen::Matrix<double, 6, 1>& error, const Eigen::MatrixXd& W_p, const Eigen::MatrixXd& W_o) {
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    int n = J.cols(); // Número de articulaciones (variables)
    Eigen::MatrixXd J_p = J.block(0, 0, 3, n);
    Eigen::MatrixXd J_o = J.block(3, 0, 3, n);
    Eigen::VectorXd e_p = error.head(3);
    Eigen::VectorXd e_o = error.tail(3);
    Eigen::SparseMatrix<double> H = Eigen::MatrixXd::Identity(n, n).sparseView();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n);

    // Formular el problema como min ||W_p * J_p * q_dot - W_p * e_p||^2 + ||W_o * J_o * q_dot - W_o * e_o||^2
    // Esto lleva a un problema de mínimos cuadrados que se puede resolver con QP.
    Eigen::MatrixXd A = (W_p * J_p).transpose() * (W_p * J_p) + (W_o * J_o).transpose() * (W_o * J_o);
    Eigen::VectorXd b = (W_p * J_p).transpose() * (W_p * e_p) + (W_o * J_o).transpose() * (W_o * e_o);
    // A es la matriz Hessiana y b es el vector de gradiente
    Eigen::SparseMatrix<double> H_qp = A.sparseView();
    Eigen::VectorXd g_qp = -b; // El solver de QP minimiza 0.5 * x^T * H * x + g^T * x 
    // x es la variable de decisión (q_dot)
    solver.data()->setNumberOfVariables(n);
    solver.data()->setNumberOfConstraints(0); // Sin restricciones de igualdad por ahora
    solver.data()->setHessianMatrix(H_qp);
    solver.data()->setGradient(g_qp);

    if (!solver.initSolver()) {
        throw std::runtime_error("Failed to initialize QP solver");
    }

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("Failed to solve QP problem");
    }
    
    return solver.getSolution();
}

Eigen::VectorXd UR5Kinematics::inverseKinematicsQP(
    const Eigen::VectorXd& q_initial,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Quaterniond& desired_orient,
    int max_iterations,
    double alpha,
    double weight_pos,
    double weight_orient)
{
    Eigen::VectorXd q = q_initial;
    const double joint_limit = PI;
    pinocchio::SE3 desired_pose(desired_orient.toRotationMatrix(), desired_pos);

    for (int i = 0; i < max_iterations; ++i) {
        pinocchio::forwardKinematics(*model_, *data_, q);
        pinocchio::updateFramePlacement(*model_, *data_, tool_frame_id_);

        Eigen::Matrix<double, 6, 1> error = computePoseError(desired_pose);

        if (error.norm() < 1e-4) {
            // std::cout << "Convergencia alcanzada en " << i << " iteraciones." << std::endl;
            return q;
        }

        pinocchio::Data::Matrix6x J(6, model_->nv);
        J.setZero();
        pinocchio::computeFrameJacobian(*model_, *data_, q, tool_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

        Eigen::MatrixXd W_p = Eigen::Matrix3d::Identity() * weight_pos;
        Eigen::MatrixXd W_o = Eigen::Matrix3d::Identity() * weight_orient;

        Eigen::VectorXd dq = solveQPIK(J, error, W_p, W_o);

        q += alpha * dq;
        for (int j = 1; j < q.size()-1; j++) {
            if (q[j] > joint_limit) {q[j] = joint_limit;}
            else if (q[j] < -joint_limit) {q[j] = -joint_limit;} 
        }
    }
    // std::cerr << "Advertencia: La cinemática inversa (QP) no convergió." << std::endl;
    return q;
}

