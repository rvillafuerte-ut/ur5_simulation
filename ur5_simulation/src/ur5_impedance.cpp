#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
//dinamica
#include <pinocchio/algorithm/crba.hpp>      // Para la matriz de inercia
#include <pinocchio/algorithm/rnea.hpp>      // Para Coriolis y gravedad
#include <pinocchio/algorithm/aba.hpp>       // Para la dinámica inversa
#include <Eigen/Dense>
#include <pinocchio/algorithm/compute-all-terms.hpp> // Para computeAllTerms
#include <pinocchio/algorithm/center-of-mass.hpp> // Para computeCentroidalMomentum (no directamente usado aquí pero útil)

#include <iostream>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ur5_impedance/impedance.hpp>
using namespace std;

template<typename T>
void dbg(const T& msg) {
    std::cout << msg << std::endl;
}

void initializeUR5(std::unique_ptr<pinocchio::Model>& model,  std::unique_ptr<pinocchio::Data>& data,
                   pinocchio::FrameIndex& tool_frame_id,      const std::string& urdf_path) {
    model = std::make_unique<pinocchio::Model>();

    auto logger = rclcpp::get_logger("UR5Kinematics");
    RCLCPP_INFO(logger, "Intentando cargar URDF desde: %s", urdf_path.c_str());

    try {
        pinocchio::urdf::buildModel(urdf_path, *model);
        RCLCPP_INFO(logger, "URDF cargado exitosamente!");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error cargando URDF: %s", e.what());
        throw;
    }

    data = std::make_unique<pinocchio::Data>(*model); // crea un puntero a los datos de pinocchio
    tool_frame_id = model->getFrameId("tool0");

    if (tool_frame_id == static_cast<pinocchio::FrameIndex>(model->nframes)) {
        RCLCPP_ERROR(logger, "Error: Marco 'tool0' no encontrado en el URDF!");
        throw std::runtime_error("Frame tool0 no encontrado");
    }
}

std::string get_file_path(const std::string& package_name, const std::string& relative_path) {
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
        return package_share_directory + "/" + relative_path;
    } catch (const std::exception& e) {
        throw std::runtime_error("No se pudo encontrar el paquete: " + package_name);
    }
}

Eigen::MatrixXd computeFullJacobianQuaternion(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const pinocchio::FrameIndex& tool_frame_id,
    const Eigen::VectorXd& q,
    double delta = 1e-8)
{
    int nq = q.size();
    Eigen::MatrixXd J_full(7, nq); // 3 pos + 4 quat

    // Estado nominal
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, tool_frame_id);
    Eigen::Vector3d pos0 = data.oMf[tool_frame_id].translation();
    Eigen::Quaterniond quat0(data.oMf[tool_frame_id].rotation());

    for (int i = 0; i < nq; ++i) {
        Eigen::VectorXd q_perturbed = q;
        q_perturbed[i] += delta;

        pinocchio::forwardKinematics(model, data, q_perturbed);
        pinocchio::updateFramePlacement(model, data, tool_frame_id);
        Eigen::Vector3d pos1 = data.oMf[tool_frame_id].translation();
        Eigen::Quaterniond quat1(data.oMf[tool_frame_id].rotation());

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


void load_values_from_file(const std::string &file_path, double values[], int size, int line_number) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("No se pudo abrir el archivo: " + file_path);
    }

    std::string line;
    int current_line = 0;
    while (std::getline(file, line)) {
        current_line++;
        if (current_line == line_number) {
            std::istringstream iss(line);
            for (int i = 0; i < size; ++i) {
                if (!(iss >> values[i])) {
                    throw std::runtime_error("Error al leer los valores desde la línea " + std::to_string(line_number));
                }
                if (iss.peek() == ',' || iss.peek() == ' ') {
                    iss.ignore(); // Ignorar comas o espacios
                }
            }
            break;
        }
    }

    if (current_line < line_number) {
        throw std::runtime_error("El archivo no contiene suficientes líneas.");
    }

    file.close();
}


Eigen::VectorXd impedanceControlPythonStyle(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const pinocchio::FrameIndex& tool_frame_id,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& dq,
    string config_path,
    double qt_[4], // Cuaternión deseado (w, x, y, z)
    double x_des[3], // Para xdes
    Eigen::MatrixXd& J_anterior,
    double dt=0.01,
    const Eigen::Vector3d& v_des = Eigen::Vector3d::Zero(), // <-- valor por defecto
    const Eigen::Vector3d& a_des = Eigen::Vector3d::Zero() 
) {
    // 1. Cinemática directa y Jacobiano actual
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::updateFramePlacement(model, data, tool_frame_id);

    

    Eigen::MatrixXd J = computeFullJacobianQuaternion(model, data, tool_frame_id, q);
    Eigen::MatrixXd dJ = (J - J_anterior) / dt; //7x6
    Eigen::VectorXd dx_current_cartesian = J * dq; // (7x6) * (6x1) = 7x1

    Eigen::Vector4d vel_ori_d = Eigen::Vector4d::Zero();
    Eigen::Vector4d acc_ori_d = Eigen::Vector4d::Zero();
    Eigen::VectorXd vel_des(7), dvel_des(7);
    vel_des << v_des, vel_ori_d;
    dvel_des << a_des, acc_ori_d;
    
    // Cargar valores de K y B desde el archivo de configuración
    double K[7]; load_values_from_file(config_path, K, 7, 23);       // 7 valores de K
    double B[7]; load_values_from_file(config_path, B, 7, 25);       // 7 valores de B

    Eigen::Matrix<double,7,7> Kp_task = Eigen::Matrix<double,7,7>::Identity();
    Eigen::Matrix<double,7,7> Kd_task = Eigen::Matrix<double,7,7>::Identity();
    Kp_task .diagonal() << K[0], K[1], K[2], K[3], K[4], K[5], K[6]; // Asigna los valores de K
    Kd_task.diagonal() << B[0], B[1], B[2], B[3], B[4], B[5], B[6]; // Asigna los valores de B
 
    Eigen::Quaterniond desired_orientation_quat(qt_[0], qt_[1], qt_[2], qt_[3]); // w, x, y, z
    desired_orientation_quat.normalize();
    
    // Error de posición
    Eigen::Vector3d position_error(x_des[0],x_des[1],x_des[2]);
    position_error-=data.oMf[tool_frame_id].translation();

    // Error de orientación usando cuaterniones
    Eigen::Quaterniond current_orientation(data.oMf[tool_frame_id].rotation());

    // Error de velocidad (vel_des - dx_current_cartesian)
    Eigen::VectorXd error_velocity = vel_des - dx_current_cartesian; // (7x1)
    Eigen::VectorXd error_pose(7);
    error_pose << -position_error, (current_orientation.coeffs() - desired_orientation_quat.coeffs());



    // 5. Cálculos de dinámica (M, c, g)
    // Necesitamos recalcular estas para cada paso de tiempo
    pinocchio::computeJointJacobians(model, data, q); // Necesario para crba y otros
    pinocchio::crba(model, data, q); // Calcula la matriz de inercia M
    pinocchio::computeCoriolisMatrix(model, data, q, dq); // Calcula el término de Coriolis
    pinocchio::computeGeneralizedGravity(model, data, q); // Calcula el vector de gravedad g

    Eigen::MatrixXd M = data.M; // Matriz de inercia
    // data.nle ya es (Coriolis + Gravity) en Pinocchio
    Eigen::VectorXd c_term = data.nle - data.g; // Solo el término de Coriolis (c en Python)
    Eigen::VectorXd g_term = data.g; // Solo el término de gravedad (g en Python)  6*1
    // cout<<submatrix<<endl;
    // if (submatrix.determinant()==0){
    //     cout<<"Jacobiano de orientación indeterminado3"<<endl;
    // }
    // 6. Calcular Md (Matriz de inercia deseada en el espacio de tarea)
    // Asegurarse de usar pseudo-inversa como en Python
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse(); // (N x 7)
    Eigen::MatrixXd Md = J * M.inverse() * J.transpose(); // (7x7) //inercia deseada en el espacio de tarea
    Eigen::MatrixXd Md_inv;
    
    Md_inv = Md.completeOrthogonalDecomposition().pseudoInverse();
    
    //MJ-¹[xdd_d-J] 

    // Torque final: TermA * TermB + TermC 
    Eigen::VectorXd tau = M*J_pinv * ((dvel_des-Kp_task*(error_pose)-Kd_task*(error_velocity))-J*dq)+data.nle;

    std::string geo_pos = get_file_path("ur5_simulation",     "launch/output_data_esfuerzo.txt");
    std::ofstream output_file_;
    output_file_.open(geo_pos, std::ios::out | std::ios::app);
    //guardar torques en aoutput_data_esfuerz.txt
    if (output_file_.is_open()) {
                output_file_ << tau[0] << " " << tau[1] << " " << tau[2] << " " << tau[3] << " " << tau[4] << " " << tau[5] <<endl;
            }

    std::string doc_vel = get_file_path("ur5_simulation",     "launch/output_data_velocidad.txt");
    std::ofstream output_file_vel_;
    output_file_vel_.open(doc_vel, std::ios::out | std::ios::app);
    Eigen::VectorXd vel_cartesiana = J * dq; // (6x1) // Torque en el espacio de velocidad
    //guardar torques en aoutput_data_esfuerz.txt
    if (output_file_vel_.is_open()) {
                output_file_vel_ << vel_cartesiana[0] << " " << vel_cartesiana[1] << " " << vel_cartesiana[2] << " " << vel_cartesiana[3] << " " << vel_cartesiana[4] << " " << vel_cartesiana[5] <<endl;
            }
    // Actualizar J_anterior para la próxima iteración
    J_anterior = J;
    dbg("Torque calculado: " + std::to_string(tau.norm()));
    pinocchio::crba(model, data, q); // Asegura que la matriz M de Pinocchio esté actualizada
    Eigen::MatrixXd M_current = (data).M;
    Eigen::MatrixXd M_inv_current;
    if (M_current.determinant() == 0) {
        M_inv_current = M_current.completeOrthogonalDecomposition().pseudoInverse();
    } else {
        M_inv_current = M_current.inverse();
    }

    // Calcula los términos no lineales C y G para el cálculo de qdd
    // Necesarios para qdd = M_inv * (tau - (C+G))
    pinocchio::computeCoriolisMatrix(model, data, q, dq);
    pinocchio::computeGeneralizedGravity(model, data, q);
    Eigen::VectorXd C_plus_G = (data).nle; // nle = Coriolis + Gravity

    // Calcula la aceleración articular deseada
    Eigen::VectorXd qdd_calculated = M_inv_current * (tau - C_plus_G);
    Eigen::VectorXd q_solution;
    Eigen::VectorXd qd_solution;
    // Integrar para obtener nuevas velocidades y posiciones articulares


    qd_solution =dq + qdd_calculated * 0.01;
    q_solution = q + qd_solution * 0.01;
    return q_solution;
}




class UR5eJointController : public rclcpp::Node {
    public:
        UR5eJointController() : Node("ur5e_joint_controller"), time_elapsed_(0.0) {
             // Inicializa con 6 elementos, todos a cero
            output_file_.open(geo_pos, std::ios::out | std::ios::app);
            load_values_from_file(config_path, q_init, 6, 7); 
            impedance_solver_ = std::make_unique<UR5Impedance>(urdf_path);
            q_des = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            q_des<< 1.0,-1.57,1.57,0,0,0; // Asigna los valores deseados

            qd_ = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            qdd_ = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero

            // q_init = Eigen::VectorXd::Zero(6); // Inicializa con 6 elementos, todos a cero
            // q_init << 0,-1.57,1.57,0,0,0; // Asigna los valores deseados


            try {
                load_values_from_file(config_path, control_loop_time, 1, 21);       // Leer la 7ma línea para q_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            //convierte el valor de control_loop_time a int
            ur5_time = static_cast<int>(control_loop_time[0]); // Convertir a milisegundos
            initializeUR5(model, data, tool_frame_id, urdf_path);            // Publicador para enviar trayectorias
            joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(control_topic, 10);
    
            
            // Suscriptor para leer el estado articular actual
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&UR5eJointController::update_joint_positions, this, std::placeholders::_1));            
                // Suscriptor para leer la posición cartesiana del haptic phantom
            
            
            timer_ = this->create_wall_timer( std::chrono::milliseconds(1000), std::bind(&UR5eJointController::posicion_inicial, this));
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(ur5_time),std::bind(&UR5eJointController::control_loop, this));

        }
    
    private:
        bool posicion_inicial_alcanzada_ = false; // Bandera para sincronización
        double time_elapsed_=0.0; // Variable

        double qt_[4]; // Orientacion del haptic phantom  
        double x_des_[3];
        Eigen::Vector3d x_init_; // Para guardar la posición inicial
        Eigen::Vector4d qt_init_; // Para guardar la posición inicial
        bool x_init_set_ = false;
        double t_traj_ = 0.0; // Tiempo para la trayectoria, independiente de dt

        std::ofstream output_file_;
        std::ofstream output_file_2;
        std::ofstream output_file_3;        
        
        Eigen::VectorXd q_ = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd q_des;
        Eigen::VectorXd qd_;
        Eigen::VectorXd qdd_;

        double q_init[6]; // Inicialización de las posiciones articulares del UR50
        double K[6];
        double B[6];
        double control_loop_time[1]; 
        int ur5_time = 0.01;
        string control_topic = "/joint_trajectory_controller/joint_trajectory";
        sensor_msgs::msg::JointState::SharedPtr last_joint_state_;    
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;
        std::unique_ptr<pinocchio::Model> model; // declarar puntero único para el modelo
        std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos  
        pinocchio::FrameIndex tool_frame_id; 
        Eigen::MatrixXd J_anterior= Eigen::MatrixXd::Zero(7, 6); // Inicializar J_anterior como una matriz de ceros
        std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5e.urdf");
        std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
        std::string geo_pos = get_file_path("ur5_simulation",     "launch/output_data_impedancia.txt");
        std::unique_ptr<UR5Impedance> impedance_solver_;

        
        

        // POSICIONES ARTICULARES DEL UR5
        void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_joint_state_ = msg;
            bool implementacion = false; // Variable para determinar si se implementa la cinemática inversa
            if (implementacion){
                q_[0] = msg->position[5];           qd_[0] = msg->velocity[5];
                q_[1] = msg->position[0];           qd_[1] = msg->velocity[0];
                q_[2] = msg->position[1];           qd_[2] = msg->velocity[1];
                q_[3] = msg->position[2];           qd_[3] = msg->velocity[2];
                q_[4] = msg->position[3];           qd_[4] = msg->velocity[3];
                q_[5] = msg->position[4];           qd_[5] = msg->velocity[4];
            }
            else{
                for (int i = 0; i < 6; ++i) {
                    q_[i] = msg->position[i];
                    qd_[i] = msg->velocity[i];

                }
            }
            // Imprimir las posiciones articulares
            //RCLCPP_INFO(this->get_logger(), "Posiciones articulares: %.4f %.4f %.4f %.4f %.4f %.4f",   q_[0], q_[1], q_[2], q_[3], q_[4], q_[5]);
            // Imprimir las velocidades articulares
            //RCLCPP_INFO(this->get_logger(), "Velocidades articulares: %.4f %.4f %.4f %.4f %.4f %.4f",  qd_[0], qd_[1], qd_[2], qd_[3], qd_[4], qd_[5]);
        }
        void posicion_inicial() {
            
            if (posicion_inicial_alcanzada_) {
                // Si ya se alcanzó la posición inicial, no hacer nada
                return;
            }
            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            RCLCPP_INFO(this->get_logger(), "Enviando posición inicial: %.2f %.2f %.2f %.2f %.2f %.2f",
                        q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]);
            point.positions = {q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]};

            point.time_from_start = rclcpp::Duration::from_seconds(4); // El tiempo de control
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);

            // Verificar si la posición actual coincide con la posición inicial
            bool posicion_correcta = true;
            for (int i = 0; i < 6; ++i) {
                cout<<"posicion del ur5"<<q_<<endl;
                cout<<"diferencia articular: "<<q_[i] - q_init[i]<<endl;
                if (std::abs(q_[i] - q_init[i]) > 0.01) { // Tolerancia de 0.01 radianes
                    posicion_correcta = false;
                    break;
                }
            }
            // imprimir la pos cartesiana del UR5
            pinocchio::forwardKinematics(*model, *data, q_); //Eigen::Map<Eigen::VectorXd>(q_, 6) -> converite duble[] a eigen::vector
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            cout<< "x: "<< data->oMf[tool_frame_id].translation()[0]<<" y: "<< data->oMf[tool_frame_id].translation()[1]<<" z: "<< data->oMf[tool_frame_id].translation()[2]<<endl; 
            //cout<< "quat: "<< data->oMf[tool_frame_id].rotation().w()<<" "<< data->oMf[tool_frame_id].rotation().x()<<" "<< data->oMf[tool_frame_id].rotation().y()<<" "<< data->oMf[tool_frame_id].rotation().z()<<endl;

            if (posicion_correcta) {
                RCLCPP_INFO(this->get_logger(), "Posición inicial alcanzada.");
                posicion_inicial_alcanzada_ = true;
            }
        }
        
        
        void control_loop() {
            if (!posicion_inicial_alcanzada_) {
                // Si no se ha alcanzado la posición inicial, no ejecutar el bucle de control
                return;
            }
            //load_values_from_file(config_path, qt_,4, 29);
            // try {
                
            //     load_values_from_file(config_path, qt_,4, 29);
            //     load_values_from_file(config_path, x_des_,3, 27);
            // } catch (const std::exception &e) {
            //     cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            // }
            if (!x_init_set_) {
                pinocchio::forwardKinematics(*model, *data, q_, qd_);
                pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
                cout<<"obteniendo primeros valores"<<endl;
                x_init_ = data->oMf[tool_frame_id].translation();
                qt_init_= Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).coeffs(); // w, x, y, z
                qt_[0]=qt_init_[3];
                qt_[1]=qt_init_[0];
                qt_[2]=qt_init_[1];
                qt_[3]=qt_init_[2];
                // Asegurarse de que qt_ esté en el orden correcto
                x_init_set_ = true;
                t_traj_ = 0.0;
            }
            cout<<"qt: "<<qt_[0]<<" "<<qt_[1]<<" "<<qt_[2]<<" "<<qt_[3]<<endl;
            cout<<"x_init: "<<x_init_[0]<<" "<<x_init_[1]<<" "<<x_init_[2]<<endl;
            // 1. Calcular la trayectoria deseada
            double c0 = 0.1;
            
            double exp_c0 = std::exp(-c0 * t_traj_);

            double x_d = x_init_.x(), y_d = x_init_.y(), z_d = x_init_.z();
            double x_dot_d = 0.0, y_dot_d = 0.0, z_dot_d = 0.0;
            double x_ddot_d = 0.0, y_ddot_d = 0.0, z_ddot_d = 0.0;

            int trayectoria = 2;
            if(trayectoria==1){
                    // Trayectoria deseada
                x_d = x_init_.x() - 0.3 + 0.3 * exp_c0;
                y_d = x_init_.y() + 0.1 - 0.1* exp_c0;
                z_d = x_init_.z() + 0.1 - 0.1 * exp_c0;

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
                double exp_neg_c0_t = std::exp(-c0 * t_traj_);
                double sin_wn_t = std::sin(wn * t_traj_);
                double cos_wn_t = std::cos(wn * t_traj_);

                // Factor de amplitud que crece de 0 a 1
                double amp_factor = 1.0 - exp_neg_c0_t;

                // Trayectoria deseada (inicia en x_init_ y la amplitud crece)
                x_d = x_init_.x() + Ax * amp_factor * sin_wn_t;
                y_d = x_init_.y() + Ay * amp_factor * cos_wn_t;
                z_d = x_init_.z() + Az * amp_factor * sin_wn_t;

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
            Eigen::Vector3d vel_d(x_dot_d, y_dot_d, z_dot_d);
            Eigen::Vector3d acc_d(x_ddot_d, y_ddot_d, z_ddot_d);
            // 2. Calcular la trayectoria deseada
            // for (int i = 0; i < 3; ++i) {
            //     x_des_[i] = x_init_[i] + 0.1;//A[i] * sin(0.05 * t_traj_) * exp(-0.05 * t_traj_);
            // }
            x_des_[0] = x_d; // Asigna la posición deseada en x
            x_des_[1] = y_d; // Asigna la posición deseada en y
            x_des_[2] = z_d; // Asigna la posición deseada en z

            // 3. Incrementar t_traj_ (por ejemplo, en 0.01 por ciclo)
            t_traj_ += 0.01;

            
            //capturar la posición actual del UR5
            
            auto start = std::chrono::high_resolution_clock::now();

            // 3. Llama a la nueva función
            // El dt de tu python es 0.01, que coincide con tu tiempo de control
            double control_dt = 0.01;

            // Asegúrate de que q_ y qd_ están actualizados desde el suscriptor antes de llamar a esto.
            // Si `last_joint_state_` no se ha recibido todavía, q_ y qd_ estarán en sus valores iniciales.
            if (!last_joint_state_) {
                RCLCPP_WARN(this->get_logger(), "No se han recibido joint_states. Usando posiciones/velocidades iniciales.");
                // Podrías añadir un 'return' o esperar hasta tener datos.
                // Para este ejemplo, asumiremos que se reciben rápidamente.
            }
            double K[7]; load_values_from_file(config_path, K, 7, 23);       // 7 valores de K
            double B[7]; load_values_from_file(config_path, B, 7, 25);       // 7 valores de B

            Eigen::Matrix<double,7,1> Kp_task = Eigen::Matrix<double,7,1>::Identity();
            Eigen::Matrix<double,7,1> Kd_task = Eigen::Matrix<double,7,1>::Identity();
            Kp_task << K[0], K[1], K[2], K[3], K[4], K[5], K[6]; // Asigna los valores de K
            Kd_task << B[0], B[1], B[2], B[3], B[4], B[5], B[6]; // Asigna los valores de B

            // Eigen::VectorXd q_solution = impedance_solver_->calculateControlCommand(
            //     q_, 
            //     qd_,                
            //     Eigen::Vector3d(x_des_[0], x_des_[1], x_des_[2]), // Para xdes
            //     Eigen::Quaterniond(qt_[0], qt_[1], qt_[2], qt_[3]),
            //     vel_d,
            //     acc_d,
            //     Kp_task,
            //     Kd_task,
            //     control_dt
            // );

            Eigen::VectorXd q_solution = impedanceControlPythonStyle(
                *model, *data, tool_frame_id, q_, qd_,
                  config_path,// Kd (Bd en Python)
                qt_, // Cuaternión deseado (w, x, y, z)
                x_des_, // Para xdes
                J_anterior, control_dt,
                vel_d, acc_d);         // Se pasará por referencia para actualizar J_anterior dentro de la función
                                        // y usarlo en el siguiente ciclo.



            for (int i = 0; i < 6; ++i) {
                if (q_solution[i] > M_PI) {
                    q_solution[i] = M_PI; // Normaliza a [-pi, pi]
                } else if (q_solution[i] < -M_PI) {
                    q_solution[i] = -M_PI; // Normaliza a [-pi, pi]
                }
            }
            auto end = std::chrono::high_resolution_clock::now();
            // ... (el resto del código para publicar la trayectoria con las nuevas q_)
            std::cout << "Tiempo total: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns" << std::endl;
            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            RCLCPP_INFO(this->get_logger(), "Enviando posición: %.4f %.4f %.4f %.4f %.4f %.4f",
                        q_solution[0], q_solution[1], q_solution[2], q_solution[3], q_solution[4], q_solution[5]);
            point.positions = {q_solution[0], q_solution[1], q_solution[2], q_solution[3], q_solution[4], q_solution[5]};
            point.time_from_start = rclcpp::Duration::from_seconds(0.01); // El tiempo de control
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);
            //comparando x_des con la posición actual del UR5
            pinocchio::forwardKinematics(*model, *data, q_, qd_);
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            pinocchio::SE3 current_pose = (*data).oMf[tool_frame_id];
            Eigen::Vector3d current_position = current_pose.translation();
            Eigen::Quaterniond current_orientation(current_pose.rotation());
            cout << "Posición deseada del UR5: " << x_des_[0] << ", " << x_des_[1] << ", " << x_des_[2] << std::endl;
            cout << "Posición actual del UR5: " << current_position.transpose() << std::endl;
            if (output_file_.is_open()) {
                output_file_ << x_d << " " << y_d << " " << z_d << " " << current_position[0] << " " << current_position[1] << " " << current_position[2] << " "
                <<qt_[0]<<" "<<qt_[1]<<" "<<qt_[2]<<" "<<qt_[3]<<" "<< current_orientation.w()<<" "<<current_orientation.x()<<" "<<current_orientation.y()<<" "<<current_orientation.z()<<endl;
            }
            // J_anterior se actualiza dentro de impedanceControlPythonStyle, no aquí
            time_elapsed_ += control_dt; // Incrementa el tiempo transcurrido
        }

    };


int main(int argc, char **argv) {  
    cout<<"Control con Geomagic"<<endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5eJointController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  
 }

