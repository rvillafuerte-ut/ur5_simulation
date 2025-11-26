#include "ur5_kinematics/kinematics.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include <iostream>

// Clase alternativa con HQP (ligero) para comparación.
// Se reutiliza el mismo modelo pero se expone una función libre que implementa el comportamiento jerárquico.

namespace ur5_hqp {

Eigen::VectorXd inverseKinematicsQPHierarchical(
    UR5Kinematics& base_solver,
    const Eigen::VectorXd& q_initial,
    const Eigen::Vector3d& desired_pos,
    const Eigen::Quaterniond& desired_orient,
    int max_iterations,
    double alpha,
    double weight_pos = 1.0,
    double weight_orient = 0.9,
    double w_posture = 1e-2)
{
    // Copia de la versión anterior con tarea secundaria (postura) para análisis comparativo
    Eigen::VectorXd q = q_initial;
    const double joint_limit = PI;
    const double dq_max_norm = 0.5;
    const Eigen::VectorXd q_nominal = Eigen::VectorXd::Zero(q.size());
    pinocchio::SE3 desired_pose(desired_orient.toRotationMatrix(), desired_pos);

    const int iter_cap = std::min(max_iterations, 15);
    const double alpha_eff = std::max(0.1, std::min(alpha, 1.0));

    for (int i = 0; i < iter_cap; ++i) {
        // FK y jacobiano a través de la API pública
        base_solver.forwardKinematics(q);
        // Acceso interno: replicar computePoseError (no público). Recalcular manualmente.
        // Para evitar acceso privado, volvemos a calcular lo necesario aquí:
        // Recuperar pose actual (ya calculada en forwardKinematics)
        // NOTA: No hay método para obtener error directamente, copiamos lógica simplificada.
        // Usamos los datos internos: se requiere modificar la clase para acceso completo si se desea más encapsulación.
        // Aquí asumimos que computePoseError era: posición + parte vector del quaternion de error.

        // Debido a que los miembros son privados, no podemos acceder directamente al data_; por lo tanto esta función
        // debería idealmente estar dentro de la clase. Si se quiere completa exactitud, mover esta implementación a un nuevo método
        // dentro de la clase. Por ahora usamos la función original inverseKinematicsQP para construir pasos intermedios.
        // Simplificación: llamamos a la versión simplificada y usamos su resultado incremental como dq1.

        Eigen::VectorXd q_step = base_solver.inverseKinematicsQP(q, desired_pos, desired_orient, 1, alpha_eff, weight_pos, weight_orient);
        Eigen::VectorXd dq1 = q_step - q;

        // Si la tarea secundaria está activa, resolver QP en nulidad
        Eigen::VectorXd dq2 = Eigen::VectorXd::Zero(q.size());
        if (w_posture > 0.0) {
            // Este bloque reusa jacobiano aproximado usando diferencia finita básica
            // (coste leve) para evitar acceso privado a J directamente.
            const double eps = 1e-6;
            pinocchio::Data::Matrix6x J(6, q.size());
            J.setZero();
            // Recalcular jacobiano vía perturbación numérica (simplificada)
            // Para precisión se recomienda exponer una función pública que devuelva J.
            for (int j = 0; j < q.size(); ++j) {
                Eigen::VectorXd q_pert = q;
                q_pert[j] += eps;
                pinocchio::SE3 pose_plus = base_solver.forwardKinematics(q_pert);
                pinocchio::SE3 pose_curr = base_solver.forwardKinematics(q);
                // Diferencias de posición
                Eigen::Vector3d dp = (pose_plus.translation() - pose_curr.translation()) / eps;
                // Diferencia de orientación (vector parte del quaternion)
                Eigen::Quaterniond q_curr(pose_curr.rotation());
                Eigen::Quaterniond q_plus(pose_plus.rotation());
                Eigen::Quaterniond q_err = q_plus * q_curr.inverse();
                Eigen::Vector3d dox = q_err.vec() / eps; // aproximación
                J.block<3,1>(0,j) = dp;
                J.block<3,1>(3,j) = dox;
            }
            // Resolver QP secundario: min ||dq2||^2 + término de postura, s.a. J*dq2 = 0
            OsqpEigen::Solver solver2;
            solver2.settings()->setVerbosity(false);
            solver2.settings()->setWarmStart(true);
            const int n = static_cast<int>(q.size());
            const int m_eq = 6;
            Eigen::SparseMatrix<double> H2(n,n);
            H2.setIdentity();
            H2 *= (w_posture + 1e-8);
            Eigen::VectorXd g2 = (w_posture) * (q + dq1 - q_nominal);
            Eigen::SparseMatrix<double> Aeq = J.sparseView();
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(m_eq);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(m_eq);
            solver2.data()->setNumberOfVariables(n);
            solver2.data()->setNumberOfConstraints(m_eq);
            solver2.data()->setHessianMatrix(H2);
            solver2.data()->setGradient(g2);
            solver2.data()->setLinearConstraintsMatrix(Aeq);
            solver2.data()->setLowerBound(lb);
            solver2.data()->setUpperBound(ub);
            if (solver2.initSolver() && solver2.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
                dq2 = solver2.getSolution();
            }
        }
        Eigen::VectorXd dq = dq1 + dq2;
        double nrm = dq.norm();
        if (nrm > dq_max_norm) dq *= (dq_max_norm / nrm);
        q.noalias() += dq;
        for (int j = 0; j < q.size(); ++j) {
            if (q[j] > joint_limit) q[j] = joint_limit; else if (q[j] < -joint_limit) q[j] = -joint_limit;
        }
    }
    return q;
}

} // namespace ur5_hqp
