#include <rclcpp/rclcpp.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ur5_dynamics_pinocchio/yaml_loader.hpp" // Asegúrate de que esta cabecera provea load_joint_positions
#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <limits>
#include <cinttypes>
#include <fstream>


class UR5DynamicsNode : public rclcpp::Node
{
public:
    UR5DynamicsNode() : Node("ur5_dynamics_node"), first_callback_(true),flag_pos1_done(true)
    {
        // --- Carga de Modelo y Configuración Inicial ---
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("ur5_dynamics_pinocchio");
        std::string urdf_path = pkg_share_dir + "/urdf/ur5e.urdf";
        std::string joint_config_path = pkg_share_dir + "/configs/joint_config.yaml";
        std::string control_params_path = pkg_share_dir + "/configs/control_params.yaml";

        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        
        nq_ = model_.nq;
        nv_ = model_.nv;

        // Frame del efector final
        end_effector_frame_id_ = model_.getFrameId("wrist_3_link");

        // --- Estados del Robot ---
        // q_current_ se carga desde joint_config.yaml. Asumimos que esta es la posición inicial.
        q_current_ = load_joint_positions(joint_config_path, nq_); 
        q_command_ = q_current_; // El comando inicial es la posición actual
        v_current_ = Eigen::VectorXd::Zero(nv_);
        v_command_ = Eigen::VectorXd::Zero(nv_);

        // Inicializar tiempo
        start_time_ = this->now();

        // --- Cargar parámetros de control ---
        loadControlParameters(control_params_path);

        // --- Configurar Waypoints PROGRAMÁTICAMENTE ---
        // NOTA: Descomenta o ajusta si quieres usar waypoints desde el YAML nuevamente.
        // Las líneas de carga de waypoints desde el YAML en loadControlParameters deben ser comentadas
        // o eliminadas si solo usas el waypoint generado aquí.

        // Usa la configuración articular inicial como base
        Eigen::VectorXd q_initial_for_waypoint = q_current_; 

        // Modifica ligeramente una o más articulaciones para crear un waypoint cercano
        // Por ejemplo, mueve el shoulder_lift_joint un poco hacia arriba.
        // Asegúrate de que los índices de las articulaciones coincidan con el orden de tu URDF.
        // UR5: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
        // Aquí modificamos la segunda articulación (shoulder_lift_joint) por 0.1 radianes.
        if (nq_ >= 2) {
            q_initial_for_waypoint[1] += 0.1; // Índice 1 para shoulder_lift_joint
        }
        // También puedes modificar la tercera articulación (elbow_joint)
        // if (nq_ >= 3) {
        //     q_initial_for_waypoint[2] -= 0.05; // Índice 2 para elbow_joint
        // }

        Eigen::VectorXd q_home(nq_);
        q_home << 0.0, -M_PI / 2.0, 0, -M_PI / 2.0, 0.0, 0.0;

        // --- Publicadores y Suscriptores de ROS ---
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&UR5DynamicsNode::jointStateCallback, this, std::placeholders::_1));

        // Ajustar dt_ para que coincida con la frecuencia de publicación
        dt_ = 0.02; // 50 Hz

        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&UR5DynamicsNode::controlLoop, this));
            
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&UR5DynamicsNode::publishTrajectoryPoint, this));

        print_data_ = this->create_wall_timer(
            std::chrono::duration<double>(1),
            std::bind(&UR5DynamicsNode::stream_data, this));

        log_file_.open("/tmp/ur5_log.txt");
        log_file_ << "t "
                << "q_command0 q_command1 q_command2 q_command3 q_command4 q_command5 "
                << "q_current0 q_current1 q_current2 q_current3 q_current4 q_current5 "
                << "pose_desired_x pose_desired_y pose_desired_z "
                << "pose_desired_qx pose_desired_qy pose_desired_qz pose_desired_qw "
                << "pose_trayectoria_x pose_trayectoria_y pose_trayectoria_z "
                << "pose_trayectoria_qx pose_trayectoria_qy pose_trayectoria_qz pose_trayectoria_qw "
                << "pose_actual_x pose_actual_y pose_actual_z "
                << "pose_actual_qx pose_actual_qy pose_actual_qz pose_actual_qw "
                << "pos_error_x pos_error_y pos_error_z "
                << "ori_error_x ori_error_y ori_error_z "
                << "v_cart_actual_x v_cart_actual_y v_cart_actual_z v_cart_actual_rx v_cart_actual_ry v_cart_actual_rz "
                << "v_cart_desired_x v_cart_desired_y v_cart_desired_z v_cart_desired_rx v_cart_desired_ry v_cart_desired_rz "
                << "tau "
                << std::endl;

        RCLCPP_INFO(this->get_logger(), "Nodo de dinámica UR5 con SMC en Espacio de Trabajo inicializado.");
    }
    ~UR5DynamicsNode() {
        if (log_file_.is_open()) {
            log_file_.close();
            RCLCPP_INFO(this->get_logger(), "Archivo de log cerrado correctamente.");
        }
    }

private:
    void loadControlParameters(const std::string& file_path) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            YAML::Node params = config["control_params"];
            
            // Cargar ganancias lambda
            std::vector<double> lambda_vec = params["lambda"].as<std::vector<double>>();
            if (lambda_vec.size() != 6) {
                RCLCPP_ERROR(this->get_logger(), "lambda debe tener 6 elementos. Usando [0.3, ...].");
                lambda_ = Eigen::VectorXd::Constant(6, 0.3);
            } else {
                lambda_ = Eigen::Map<Eigen::VectorXd>(lambda_vec.data(), 6);
            }
            
            // Cargar ganancias k
            std::vector<double> k_vec = params["k"].as<std::vector<double>>();
            if (k_vec.size() != 6) {
                RCLCPP_ERROR(this->get_logger(), "k debe tener 6 elementos. Usando [0.6, ...].");
                k_ = Eigen::VectorXd::Constant(6, 0.6);
            } else {
                k_ = Eigen::Map<Eigen::VectorXd>(k_vec.data(), 6);
            }
            std::vector<double> k2_vec = params["k2"].as<std::vector<double>>();
            k2_ = Eigen::Map<Eigen::VectorXd>(k2_vec.data(), 6);

            // Cargar otros parámetros
            boundary_ = params["boundary"].as<double>(0.1);
            damping_factor_ = params["damping_factor"].as<double>(0.05);
            waypoint_tolerance_ = params["waypoint_tolerance"].as<double>(0.01);
            initial_torque_boost_ = params["initial_torque_boost"].as<double>(0.0); // No usado
            torque_boost_duration_ = params["torque_boost_duration"].as<double>(0.0); // No usado
            dt_ = 0.02; 

            if (params["safety_limits"]) {
                max_joint_velocity_ = params["safety_limits"]["max_joint_velocity"].as<double>(3.0);
            } else {
                max_joint_velocity_ = 3.0;
            }

            RCLCPP_INFO(this->get_logger(), "Parámetros de control cargados desde: %s", file_path.c_str());
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error al cargar parámetros YAML: %s. Usando valores por defecto.", e.what());
            // Valores por defecto si falla la carga
            lambda_ = Eigen::VectorXd::Constant(6, 0.3);
            k_ = Eigen::VectorXd::Constant(6, 0.6);
            boundary_ = 0.1;
            waypoint_tolerance_ = 0.01;
            dt_ = 0.02;
            max_joint_velocity_ = 3.0;
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        bool implementacion =true;
        if(implementacion){
            q_current_ << msg->position[5], msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4];
            v_current_ << msg->velocity[5], msg->velocity[0], msg->velocity[1], msg->velocity[2], msg->velocity[3], msg->velocity[4];
        }
        else{
            for (int i = 0; i < 6; ++i) {
                q_current_[i] = msg->position[i];
                v_current_[i] = msg->velocity[i];
            }
        }
        
        if (first_callback_) {
            RCLCPP_INFO(this->get_logger(), "Primera lectura de estado recibida. Iniciando control.");

            q_command_ = q_current_;
            v_command_ = v_current_;

            Eigen::VectorXd q_offset(nq_);
            q_offset.setZero();
            if (nq_ >= 6) {
                q_offset[0] = 0;
                q_offset[1] = 0;
                q_offset[2] = 0;
                q_offset[3] = 0;
                q_offset[4] = 0;
                q_offset[5] = 0;
            }
            Eigen::VectorXd q_desired_offset = q_current_ + q_offset;

            pinocchio::forwardKinematics(model_, data_, q_desired_offset);
            pinocchio::updateFramePlacement(model_, data_, end_effector_frame_id_);
            
            pose_desired_ = data_.oMf[end_effector_frame_id_];

            pinocchio::forwardKinematics(model_, data_, q_current_);
            pinocchio::updateFramePlacement(model_, data_, end_effector_frame_id_);
            pinocchio::SE3 current_end_effector_pose = data_.oMf[end_effector_frame_id_];


            RCLCPP_INFO(this->get_logger(), "Waypoint ACTUAL (robot start) (x,y,z): %f, %f, %f",
                    current_end_effector_pose.translation().x(),
                    current_end_effector_pose.translation().y(),
                    current_end_effector_pose.translation().z());
            
            RCLCPP_INFO(this->get_logger(), "Waypoint DESEADO (desde offset articular) (x,y,z): %f, %f, %f",
                    pose_desired_.translation().x(),
                    pose_desired_.translation().y(),
                    pose_desired_.translation().z());



            first_callback_ = false;
        }
    }

    void stream_data(){
        RCLCPP_INFO_STREAM(this->get_logger(), "error = " << time_error_.transpose());
        //RCLCPP_INFO_STREAM(this->get_logger(), "error = " << time_error_.cwiseAbs().transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "pose_desired = " << pose_desired_.translation().transpose());

    }

    void controlLoop()
    {
        if (first_callback_) {
            return;
        }

        try {
            pinocchio::computeAllTerms(model_, data_, q_current_, v_current_);
            pinocchio::forwardKinematics(model_, data_, q_current_);
            pinocchio::updateFramePlacements(model_, data_);

            const pinocchio::SE3& current_pose = data_.oMf[end_effector_frame_id_];

            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, nv_);
            pinocchio::computeFrameJacobian(model_, data_, q_current_, end_effector_frame_id_, 
                                          pinocchio::LOCAL_WORLD_ALIGNED, J);
            
            Eigen::MatrixXd J_dot = Eigen::MatrixXd::Zero(6, nv_);
            pinocchio::getFrameJacobianTimeVariation(model_, data_, end_effector_frame_id_, 
                                                     pinocchio::LOCAL_WORLD_ALIGNED, J_dot);

            // Guardar la posición y orientación deseada (como punto base de la trayectoria)
            Eigen::Vector3d pos0 = pose_desired_.translation();
            Eigen::Matrix3d R0 = pose_desired_.rotation();

            // Tiempo
            double t = (this->now() - start_time_).seconds();

            // Parámetros de trayectoria
            double c0 = 0.065;
            double exp_c0 = std::exp(-c0 * t);
            

            // Declarar las variables ANTES de los bloques if/else con valores por defecto
            double x_d = pos0.x(), y_d = pos0.y(), z_d = pos0.z();
            double x_dot_d = 0.0, y_dot_d = 0.0, z_dot_d = 0.0;
            double x_ddot_d = 0.0, y_ddot_d = 0.0, z_ddot_d = 0.0;

            int trayectoria = 1;
            if(trayectoria==1){
                    // Trayectoria deseada
                x_d = pos0.x() - 0.3 + 0.3 * exp_c0;
                y_d = pos0.y() + 0.1 - 0.1* exp_c0;
                z_d = pos0.z() + 0.1 - 0.1 * exp_c0;

                // Velocidad deseada
                x_dot_d = 0.3 * ( -c0 * exp_c0 );
                y_dot_d = -0.1 * ( -c0 * exp_c0 );
                z_dot_d = -0.1 * ( -c0 * exp_c0 );

                // Aceleración deseada
                x_ddot_d = 0.3 * exp_c0 * (c0*c0 );
                y_ddot_d = -0.1 * exp_c0 * (c0*c0 );
                z_ddot_d = -0.1 * exp_c0 * (c0*c0 );

            } else if (trayectoria==2) { 
                double wn = 0.5;   // Frecuencia angular (rad/s)
                double Ax = 0.05, Ay = 0.05, Az = 0.02; // Amplitudes máximas

                // Términos comunes para eficiencia
                double exp_neg_c0_t = std::exp(-c0 * t);
                double sin_wn_t = std::sin(wn * t);
                double cos_wn_t = std::cos(wn * t);

                // Factor de amplitud que crece de 0 a 1
                double amp_factor = 1.0 - exp_neg_c0_t;

                // Trayectoria deseada (inicia en pos0 y la amplitud crece)
                x_d = pos0.x() + Ax * amp_factor * sin_wn_t;
                y_d = pos0.y() + Ay * amp_factor * cos_wn_t;
                z_d = pos0.z() + Az * amp_factor * sin_wn_t;

                // --- Derivadas para la nueva trayectoria ---

                // Velocidad deseada
                double d_amp_factor_dt = c0 * exp_neg_c0_t; // Derivada del factor de amplitud
                x_dot_d = Ax * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);
                y_dot_d = Ay * (d_amp_factor_dt * cos_wn_t - amp_factor * wn * sin_wn_t);
                z_dot_d = Az * (d_amp_factor_dt * sin_wn_t + amp_factor * wn * cos_wn_t);

                // Aceleración deseada
                double d2_amp_factor_dt2 = -c0 * c0 * exp_neg_c0_t; // Segunda derivada del factor de amplitud
                double term1_sin = d2_amp_factor_dt2 - amp_factor * wn * wn;
                double term2_cos = 2 * d_amp_factor_dt * wn;

                x_ddot_d = Ax * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
                y_ddot_d = Ay * (term1_sin * cos_wn_t - term2_cos * sin_wn_t);
                z_ddot_d = Az * (term1_sin * sin_wn_t + term2_cos * cos_wn_t);
            }
            

            // Vectores Eigen
            Eigen::Vector3d pos_d(x_d, y_d, z_d);
            Eigen::Vector3d vel_d(x_dot_d, y_dot_d, z_dot_d);
            Eigen::Vector3d acc_d(x_ddot_d, y_ddot_d, z_ddot_d);

            Eigen::Vector3d vel_ori_d = Eigen::Vector3d::Zero();
            Eigen::Vector3d acc_ori_d = Eigen::Vector3d::Zero();
            Eigen::VectorXd v_desired(6), a_desired(6);
            v_desired << vel_d, vel_ori_d;
            a_desired << acc_d, acc_ori_d;

            Eigen::Matrix3d R_d = R0; // Mantener orientación constante

            // Nueva pose objetivo para el efector final
            pinocchio::SE3 pose_trayectoria(R_d, pos_d);
            //RCLCPP_INFO_STREAM(this->get_logger(), "x_d = " << x_d << ", y_d = " << y_d << ", z_d = " << z_d << ", t = " << t << pos0.transpose() 
            //<< "k"<<k_.transpose() << "k2" << k2_.transpose() << "lambda" << lambda_.transpose());

            // Error posición y orientación (igual que antes)
            Eigen::Vector3d pos_error = current_pose.translation() - pose_trayectoria.translation();
            Eigen::Matrix3d R_error = current_pose.rotation() * pose_trayectoria.rotation().transpose();
            Eigen::Vector3d ori_error = pinocchio::log3(R_error);
            Eigen::VectorXd e(6);
            e << pos_error, ori_error;
            time_error_ = e;
            //RCLCPP_INFO_STREAM(this->get_logger(), "error = " << e.transpose());

            Eigen::VectorXd v_cartesian = J * v_current_;
            // e_dot = v_actual - v_desired (assuming v_desired = 0 for waypoint)
            Eigen::VectorXd e_v = v_cartesian-v_desired; 
            
            // Superficie
            Eigen::VectorXd s = e_v + lambda_.cwiseProduct(e);


            double alpha = 10.0; 

            Eigen::VectorXd s_sign = (alpha * s.array()).tanh().matrix();
            

            Eigen::VectorXd a_cartesian_desired = a_desired-k2_.cwiseProduct(s_sign)-k_.cwiseProduct(s) - lambda_.cwiseProduct(e_v); 
            
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
            Eigen::MatrixXd JJT_damped = J * J.transpose() + damping_factor_ * damping_factor_ * I;
            Eigen::MatrixXd J_pinv = J.transpose() * JJT_damped.inverse();
            
            // q_ddot_desired = J_pinv * (a_cartesian_desired - J_dot * v_current_)
            Eigen::VectorXd q_ddot_desired = J_pinv * (a_cartesian_desired - J_dot * v_current_);
            Eigen::MatrixXd M = data_.M;
            Eigen::VectorXd Cq = data_.C * v_current_;
            Eigen::VectorXd G = data_.g;
            Eigen::VectorXd tau_smc_ = M*q_ddot_desired + Cq + G;
            RCLCPP_INFO_STREAM(this->get_logger(), "tau_smc = " << tau_smc_.transpose());
            //RCLCPP_INFO_STREAM(this->get_logger(), "s = " << s.transpose());
            //RCLCPP_INFO_STREAM(this->get_logger(), "J_dot = " << J_dot);
            //RCLCPP_INFO_STREAM(this->get_logger(), "current_pose = " << current_pose.translation());
            //RCLCPP_INFO_STREAM(this->get_logger(), "pose_desired_ = " << pose_desired_.translation());
   
            v_command_ = v_current_ + q_ddot_desired * dt_; 

            for (int i = 0; i < nv_; ++i) {
                v_command_[i] = std::max(-max_joint_velocity_, std::min(max_joint_velocity_, v_command_[i]));
            }
            q_command_ = q_current_ + v_command_ * dt_;
            //RCLCPP_INFO_STREAM(this->get_logger(), "q = " << q_command_.transpose());
            if (pos_error.norm() < 1e-2 && ori_error.norm() < 18e-3 && flag_pos1_done){
                rclcpp::Time current_time = this->now();
                rclcpp::Duration elapsed_time = current_time - start_time_;
                
                /*RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Posición alcanzada1 Norma error pos: " << pos_error.norm() 
                    << ", Norma error ori: " << ori_error.norm()
                    << ". Tiempo transcurrido: " << elapsed_time.seconds() << "segundos error menor a 1 cm y 1 grado");
                */
                //this->publish_timer_->cancel();
                flag_pos1_done = true;
                
            }
            if (pos_error.norm() < 1e-4 && ori_error.norm() < 2e-3){
                rclcpp::Time current_time = this->now();
                rclcpp::Duration elapsed_time = current_time - start_time_;
                
                RCLCPP_INFO_STREAM(this->get_logger(), 
                    "Posición alcanzada2 Norma error pos: " << pos_error.norm() 
                    << ", Norma error ori: " << ori_error.norm()
                    << ". Tiempo transcurrido: " << elapsed_time.seconds() << " segundos error menor a 0.01 cm y 0.1 grados");
                
                // end bucle de control para no seguir publicando mensajes
                //this->control_timer_->cancel();
                //this->print_data_->cancel();
                //this->publish_timer_->cancel();
                
            }
            // Quaternion para orientaciones
            Eigen::Quaterniond q_des(pose_desired_.rotation());
            Eigen::Vector3d pos_des = pose_desired_.translation();

            Eigen::Quaterniond q_traj(pose_trayectoria.rotation());
            Eigen::Vector3d pos_traj = pose_trayectoria.translation();

            Eigen::Quaterniond q_act(current_pose.rotation());
            Eigen::Vector3d pos_act = current_pose.translation();

            log_file_ << t << " ";
            for(int i=0; i<6; i++) log_file_ << q_command_[i] << " ";
            for(int i=0; i<6; i++) log_file_ << q_current_[i] << " ";
            log_file_ << pos_des.x() << " " << pos_des.y() << " " << pos_des.z() << " "
                    << q_des.x() << " " << q_des.y() << " " << q_des.z() << " " << q_des.w() << " ";
            log_file_ << pos_traj.x() << " " << pos_traj.y() << " " << pos_traj.z() << " "
                    << q_traj.x() << " " << q_traj.y() << " " << q_traj.z() << " " << q_traj.w() << " ";
            log_file_ << pos_act.x() << " " << pos_act.y() << " " << pos_act.z() << " "
                    << q_act.x() << " " << q_act.y() << " " << q_act.z() << " " << q_act.w() << " ";
            log_file_ << pos_error.x() << " " << pos_error.y() << " " << pos_error.z() << " ";
            log_file_ << ori_error.x() << " " << ori_error.y() << " " << ori_error.z() << " ";
            for(int i=0; i<6; ++i) log_file_ << v_cartesian[i] << " ";
            for(int i=0; i<6; ++i) log_file_ << v_desired[i] << " ";
            for(int i=0; i<6; ++i) log_file_ << tau_smc_[i] << " ";
            log_file_ << std::endl;


        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Excepción en controlLoop: %s", e.what());
        }
    }

    void publishTrajectoryPoint()
    {
        if (first_callback_) {
            return; 
        }
        auto traj_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
        traj_msg->joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(q_command_.data(), q_command_.data() + q_command_.size());
        point.time_from_start = rclcpp::Duration::from_seconds(dt_); 
        
        traj_msg->points.push_back(point);
        publisher_->publish(std::move(traj_msg));
    }

    // Modelo y datos de Pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;
    int nq_, nv_;
    pinocchio::FrameIndex end_effector_frame_id_;

    // Objetivo cartesiano
    pinocchio::SE3 pose_desired_;
    std::vector<pinocchio::SE3> waypoints_;
    int current_waypoint_;
    double waypoint_tolerance_;

    // Estados del robot
    Eigen::VectorXd q_current_, v_current_; // Estado medido
    Eigen::VectorXd q_command_;             // Estado de comando a publicar
    Eigen::VectorXd v_command_;

    Eigen::VectorXd time_error_;

    // Tiempo ROS
    rclcpp::Time start_time_;

    // Parámetros de control
    Eigen::VectorXd lambda_, k_,k2_;
    double dt_;
    double boundary_;
    double damping_factor_;
    double initial_torque_boost_;
    double torque_boost_duration_;
    double max_joint_velocity_;
    Eigen::VectorXd v_filtered_;
double filter_alpha_ = 0.85; 
    //data 
    std::ofstream log_file_;
    // ROS
    bool first_callback_;
    bool flag_pos1_done;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_, publish_timer_,print_data_;
};

Eigen::VectorXd load_joint_positions(const std::string& file_path, int nq) {
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        std::vector<double> positions = config["joint_positions"].as<std::vector<double>>();
        if (positions.size() != static_cast<size_t>(nq)) {
            RCLCPP_ERROR(rclcpp::get_logger("UR5DynamicsNode"), "El número de posiciones articulares en joint_config.yaml no coincide con el modelo.");
            return Eigen::VectorXd::Zero(nq); // Fallback a ceros
        }
        return Eigen::Map<Eigen::VectorXd>(positions.data(), nq);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UR5DynamicsNode"), "Error al cargar joint_config.yaml: %s. Usando posiciones articulares cero.", e.what());
        return Eigen::VectorXd::Zero(nq); // Fallback a ceros
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5DynamicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}