#include "functions.hpp"
#include <ur5_kinematics/kinematics.hpp>

string ur = "";

class UR5eJointController : public rclcpp::Node {
    public:
        UR5eJointController() : Node("ur5e_joint_controller"), time_elapsed_(0.0) {
            output_file_.open(geo_pos, std::ios::out | std::ios::app);
            output_file_2.open(control_pos, std::ios::out | std::ios::app);
            output_file_3.open(ur5_pos, std::ios::out | std::ios::app);

            initializeUR5(model, data, tool_frame_id, urdf_path);
            
            //llama constantes de config
            try {
                // Se elimina la carga de q_init desde el archivo
                load_values_from_file(config_path, controlador, 1, 35);       // Leer controlador
                load_values_from_file(config_path, qt_init_geo, 4, 11);       // Leer qt_init del GeomagicTouch
                kinematics_solver_ = std::make_unique<UR5Kinematics>(urdf_path);

            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                
            if (controlador[0] == 1) {
                control_loop_time = 10; // 10 ms
                ur5_time = 0.1; // 100 ms
            } else if (controlador[0] == 2) {
                control_loop_time = 1; // 1 ms
                ur5_time = 0.01; // 10 ms
            } else if (controlador[0] == 3) {
                control_loop_time = 1; // 1 ms
            } else {
                RCLCPP_ERROR(this->get_logger(), "Controlador no válido. Debe ser 1, 2 o 3.");
            }
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            
            if (!output_file_.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo para guardar los datos.");
            }
            // Publicador para enviar trayectorias
            joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(control_topic, 10);
    
            // Suscriptor para leer el estado articular actual
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&UR5eJointController::update_joint_positions, this, std::placeholders::_1));            
                // Suscriptor para leer la posición cartesiana del haptic phantom
            subscription_haptic_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/phantom/pose", 10, std::bind(&UR5eJointController::pose_callback, this, std::placeholders::_1));
            subscription_phantom_joints_ = this->create_subscription<sensor_msgs::msg::JointState>("/phantom/joint_states", 10, std::bind(&UR5eJointController::phantom_joint_states_callback, this, std::placeholders::_1));
            subscription_phantom_button_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>( "/phantom/button",    10,  std::bind(&UR5eJointController::button_callback, this, std::placeholders::_1));
            // Se elimina timer2_
            timer_ = this->create_wall_timer( std::chrono::milliseconds(control_loop_time), std::bind(&UR5eJointController::control_loop, this));
        }
    
    private:
        bool posicion_inicial_alcanzada_ = false; // Se activa cuando se captura la pose
        bool pose_inicial_capturada_ = false;     // Nueva bandera para asegurar una sola captura
        bool capturar_pose_inicial_ = false; // Bandera para capturar pose inicial del Geomagic
        double time_elapsed_;
        double previous_error_[3] = {0, 0, 0}; // Error anterior
        double error_[3] = {0, 0, 0}; // Error actual
        double r_[3] = {0, 0, 0}; // Posiciones del haptic phantom
        double qt_[4] = {0, 0, 0, 0}; // Orientacion del haptic phantom  
        double qt_inv[4] = {0, 0, 0, 0}; // Orientacion del haptic phantom inverso
        std::ofstream output_file_;
        std::ofstream output_file_2;
        std::ofstream output_file_3;

        Eigen::Vector3d euler_angles;
        Eigen::Matrix3d rotation_matrix;
        Eigen::Quaterniond q_x;        Eigen::Quaterniond q_y;        Eigen::Quaterniond q_z;        
        Eigen::Quaterniond quat_initial_UR5; 
        Eigen::Quaterniond quat_initial_geo; 
        Eigen::Vector3d r_initial_geo;
        Eigen::Quaterniond quat_real_geo; 
        
        Eigen::MatrixXd J_anterior= Eigen::MatrixXd::Zero(7, 6);

        Eigen::VectorXd q_ = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qd_ = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd q_solution = Eigen::VectorXd::Zero(6);

        double h_[6] ;
        double q_init[6];
        double x_init[3]; 
        double x_des[3];
        double qt_init_ur5[4];
        double qt_init_geo[4];
        int control_loop_time = 1; 
        int ur5_time;
        double max_iteraciones[1];
        double alpha[1];
        double controlador[1];
        
        sensor_msgs::msg::JointState::SharedPtr last_joint_state_;    
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_haptic_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_phantom_joints_;
        rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr subscription_phantom_button_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<pinocchio::Model> model; // declarar puntero único para el modelo
        std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos
        pinocchio::FrameIndex tool_frame_id;
        std::unique_ptr<UR5Kinematics> kinematics_solver_;
        std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
        std::string urdf_path = get_file_path("ur5_simulation", "include/" + ur + ".urdf");
        std::string ur5_pos = get_file_path("ur5_simulation",     "launch/output_data3.txt");
        std::string control_pos = get_file_path("ur5_simulation",     "launch/output_data2.txt");
        std::string geo_pos = get_file_path("ur5_simulation",     "launch/output_data.txt");
        
        
        
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "Mensaje nulo recibido en /phantom/pose.");
                return;
            }
            
            r_[0] = msg->pose.position.x;      r_[1] = msg->pose.position.y;       r_[2] = msg->pose.position.z;
   
            quat_real_geo.w() = msg->pose.orientation.w;
            quat_real_geo.x() = msg->pose.orientation.x;
            quat_real_geo.y() = msg->pose.orientation.y;
            quat_real_geo.z() = msg->pose.orientation.z;           
                        
        }    
            

        // POSICIONES ARTICULARES DEL UR5
        void update_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_joint_state_ = msg;
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

            // Capturar la pose inicial automáticamente en la primera recepción
            if (!pose_inicial_capturada_) {
                RCLCPP_INFO(this->get_logger(), "Capturando la pose inicial actual del robot...");
                
                // Copiar posiciones actuales como la posición inicial de referencia (q_init)
                for (int i = 0; i < 6; ++i) {
                    q_init[i] = q_[i];
                }
                
                // Calcular la pose cartesiana inicial (x_init, qt_init_ur5) a partir de q_init
                pinocchio::forwardKinematics(*model, *data, Eigen::Map<Eigen::VectorXd>(q_init, 6));
                pinocchio::updateFramePlacement(*model, *data, tool_frame_id);

                x_init[0] = data->oMf[tool_frame_id].translation()[0];
                x_init[1] = data->oMf[tool_frame_id].translation()[1];
                x_init[2] = data->oMf[tool_frame_id].translation()[2];

                qt_init_ur5[0] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).w();
                qt_init_ur5[1] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).x();
                qt_init_ur5[2] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).y();
                qt_init_ur5[3] = Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).z();

                pose_inicial_capturada_ = true;
                posicion_inicial_alcanzada_ = true; // Habilitar el bucle de control
                
                if (geomagic) {
                    capturar_pose_inicial_ = true;  // Habilitar la captura de pose del Geomagic
                }
                
                RCLCPP_INFO(this->get_logger(), "Pose inicial capturada. El control está activo.");
                RCLCPP_INFO(this->get_logger(), "q_init: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
                           q_init[0], q_init[1], q_init[2], q_init[3], q_init[4], q_init[5]);
            }
        }
        
        void phantom_joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Ejemplo: imprimir las posiciones articulares del haptic
            //RCLCPP_INFO(this->get_logger(), "Phantom joint states recibidas:");
            
            
            for (int i = 0; i < 6; ++i) {
                    h_[i] = msg->position[i];
                }
            q_x.w() =  std::cos((h_[4] + 3.1416)/ 4); q_x.x() =  std::sin((h_[4]+3.1416) / 4);q_x.y() =  0; q_x.z() =  0; //eje real: 
            //q_y.w() =  std::cos(euler_angles[1] / 4); q_y.x() =  0; q_y.y() =  std::sin(euler_angles[1] / 4); q_y.z() =  0;
            //q_z.w() =  std::cos(euler_angles[2] / 4); q_z.x() =  0; q_z.y() =  0; q_z.z() =  std::sin(euler_angles[2] / 4);
            
        }
        
        void button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg){
            if (msg->grey_button == 1 && capturar_pose_inicial_) {
                RCLCPP_INFO(this->get_logger(), "Botón gris presionado: Capturando pose inicial del Geomagic.");
                for (int i = 0; i < 3; ++i) {
                    r_initial_geo[i] = r_[i]; // Guarda la posición inicial del Geomagic
                }
                // Guarda la posición y orientación actuales del Geomagic
                // Si tienes orientación, aquí puedes guardar el valor correcto
                // Si tienes orientación en quat_real_geo:
                qt_init_geo[0] = quat_real_geo.w();
                qt_init_geo[1] = quat_real_geo.x();
                qt_init_geo[2] = quat_real_geo.y();
                qt_init_geo[3] = quat_real_geo.z();

                capturar_pose_inicial_ = false; // Solo captura una vez
            }
            if (msg->grey_button == 1) {
                RCLCPP_INFO(this->get_logger(), "Botón gris presionado");
            }
            if (msg->white_button == 1) {
                RCLCPP_INFO(this->get_logger(), "Botón blanco presionado");
            }
        }

        // La función posicion_inicial() se ha eliminado por completo.
        
        //Bucle de control que calcula y publica nuevas posiciones articulares.
        void control_loop() {
            if (!posicion_inicial_alcanzada_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Esperando la primera lectura de /joint_states para capturar la pose inicial...");
                return;
            }
            
            // Si se usa Geomagic, esperar a que se presione el botón para capturar su pose
            if (geomagic && capturar_pose_inicial_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Esperando captura de pose inicial del Geomagic (botón gris)...");
                return;
            }

            quat_initial_geo.w() = qt_init_geo[0]; quat_initial_geo.x() = qt_init_geo[1]; quat_initial_geo.y() = qt_init_geo[2]; quat_initial_geo.z() = qt_init_geo[3];
            
            // variables modificables desde config.txt
            try {
                load_values_from_file(config_path, max_iteraciones, 1, 18);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, alpha, 1, 16);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            
            // El cálculo de x_init y qt_init_ur5 ya no es necesario aquí

            Eigen::Quaterniond quat_trans_geo;
            cout<<"quat_real_geo: "<<quat_real_geo.w()<<" "<<quat_real_geo.x()<<" "<<quat_real_geo.y()<<" "<<quat_real_geo.z()<<endl;
            cout<<"quat_initial_geo: "<<quat_initial_geo.w()<<" "<<quat_initial_geo.x()<<" "<<quat_initial_geo.y()<<" "<<quat_initial_geo.z()<<endl;    
            quat_trans_geo = quat_real_geo * quat_initial_geo.inverse();
            cout<<"quat_trans_geo: "<<quat_trans_geo.w()<<" "<<quat_trans_geo.x()<<" "<<quat_trans_geo.y()<<" "<<quat_trans_geo.z()<<endl;
            
            Eigen::AngleAxisd aa(quat_trans_geo);
            double angle = aa.angle();
            Eigen::Vector3d axis = aa.axis();
            //axis.x() = -axis.x();
            //axis.y() = -axis.y();

            // Escalar el ángulo (por ejemplo, a la mitad)
            double escala = 0.5;
            Eigen::AngleAxisd aa_escalado(angle * escala, axis);
            

            // Reconstruir el cuaternión escalado
            quat_trans_geo = Eigen::Quaterniond(aa_escalado);

            
            cout<<"x_init: "<<x_init[0]<<" "<<x_init[1]<<" "<<x_init[2]<<endl;
            cout<<"r_init: "<<r_[0]<<" "<<r_[1]<<" "<<r_[2]<<endl;
            time_elapsed_ += 0.01; // Incremento de 100 ms
            if (geomagic) {
                int escala = 2.5;
                // Si se está usando el haptic phantom, actualizar las posiciones
                x_des[0] = (r_[1]-r_initial_geo[1])*escala + x_init[0]; // -r_[0]*2+0.647514 //0.0881142
                x_des[1] = (x_init[1]- (r_[0]-r_initial_geo[0])*escala);
                x_des[2] = (r_[2]-r_initial_geo[2])*escala + x_init[2];
            }else {                
                x_des[0] = x_init[0] + 0.1 * cos(2 * PI * 0.05 * time_elapsed_)* exp(-0.05 * time_elapsed_); //-0.10912;//x_init[0] + 0.1 * cos(2 * PI * 0.5 * time_elapsed_); // X_BASE + AMPLITUDE * cos(2 * PI * FREQUENCY * time_elapsed_)
                x_des[1] = x_init[1] + 0.1 * sin(2 * PI * 0.05 * time_elapsed_)* exp(-0.05 * time_elapsed_); //0.9479;//x_init[1] + 0.1 * sin(2 * PI * 0.5 * time_elapsed_);                                           // Y_CONST
                x_des[2] = x_init[2] + 0.1 * sin(2 * PI * 0.05 * time_elapsed_)* exp(-0.05 * time_elapsed_);//0.187537;//x_init[2];//.5 + 0.1 * sin(2 * PI * 0.5 * time_elapsed_)                                          // Z_CONST

            }
            
            //x_init[0] = -0.1152; x_init[1] = 0.493; x_init[2] =  0.293;
            
            
            cout<<"x_des: "<<x_des[0]<<" "<<x_des[1]<<" "<<x_des[2]<<endl;
            
            
            // Coordenadas deseadas (xd)
            
            // guarda la posicion del haptico en el archivo
            if (output_file_.is_open()) {
                output_file_ << r_[0] << " " << r_[1] << " " << r_[2] << " ";
                output_file_ <<   quat_trans_geo.w() << " " << quat_trans_geo.x() << " " << quat_trans_geo.y() << " " << quat_trans_geo.z() << "\n";

            }


            //convierte la pocision articular a acartesiana y quaternion
            pinocchio::forwardKinematics(*model, *data, q_);
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            // Imprimir la posición cartesiana del UR5
            

            // guarda la posicion del ur5 en el archivo
            if (output_file_3.is_open()) {
                output_file_3 << "Positions ur: "<<data->oMf[tool_frame_id].translation()[0]<<" "<<data->oMf[tool_frame_id].translation()[1]<<" "<<data->oMf[tool_frame_id].translation()[2]<<" ";  
                output_file_3 << "Orientation (quaternion): "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).w()<<" "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).x()<<" "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).y()<<" "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).z()<<endl; 

                // for (int i = 0; i < 6; ++i) {
                //     output_file_3 << q_[i] << " ";
                // }
                // output_file_3 << "\n";
            }

            

            //quat_initial.w() = -0.26707; quat_initial.x() = 0.962872; quat_initial.y() = 0.010197; quat_initial.z() = -0.03804;
            quat_initial_UR5.w() = qt_init_ur5[0]; quat_initial_UR5.x() = qt_init_ur5[1]; quat_initial_UR5.y() = qt_init_ur5[2]; quat_initial_UR5.z() = qt_init_ur5[3];
            Eigen::Quaterniond current_orientation;

            if (orientacion) {
                current_orientation = quat_initial_UR5*quat_trans_geo;//*q_x*q_x;// * q_y * q_z;
    
            } else {
                current_orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
            }
            
            double rot_des[4] = {current_orientation.w(), current_orientation.x(), current_orientation.y(), current_orientation.z()};
            
            if (controlador[0] == 1) {
                // Controlador de impedancia
                cout<<"Controlador de impedancia"<<endl;
                q_solution = impedanceControlPythonStyle(*model,*data,tool_frame_id,q_,qd_,config_path,rot_des,x_des,0.01,J_anterior);
                if (!q_solution.allFinite()) {
                    RCLCPP_ERROR(this->get_logger(), "La solución de IK no es válida.");
                    return;
                }
            } else if (controlador[0] == 2) {
                // Cinemática inversa
                cout<<"Cinemática inversa"<<endl;

                // 1. Convertir los arrays a los tipos de Eigen correctos
                Eigen::Vector3d desired_pos_vec(x_des[0], x_des[1], x_des[2]);
                Eigen::Quaterniond desired_orient_quat(rot_des[0], rot_des[1], rot_des[2], rot_des[3]); // w, x, y, z
                desired_orient_quat.normalize(); // Es una buena práctica normalizar el cuaternión

                // 2. Llamar a la función con los argumentos correctos
                q_solution = kinematics_solver_->inverseKinematicsQP(
                    q_,                                     // Posición articular actual (Eigen::VectorXd)
                    desired_pos_vec,                        // Posición deseada (Eigen::Vector3d)
                    desired_orient_quat,                    // Orientación deseada (Eigen::Quaterniond)
                    static_cast<int>(max_iteraciones[0]),   // Máximas iteraciones (int)
                    alpha[0]                                // Tasa de aprendizaje (double)
                );

                if (!q_solution.allFinite()) {
                    RCLCPP_ERROR(this->get_logger(), "La solución de IK no es válida.");
                    return;
                }

            }
            // Verificar si la solución es válida
           

            // if ((q_-q_solution).norm() > 1) {
            //     RCLCPP_ERROR(this->get_logger(), "La diferencia entre la posición inicial y la solución es demasiado grande.");
            //     q_solution = q_;
                
            // }
            
            if ((q_-q_solution).norm() < 0.001) {       // si la diferencia entre la posición inicial y la solución es muy pequeña, se usa la posición actual         
                q_solution = q_;

                
            }else{
                ur5_time += 0.01 * (q_ - q_solution).norm();
                cout<<"Tiempo de control: "<<ur5_time<<endl;
            }
            cout<<"q_enviado"<<q_solution[0]<<" "<<q_solution[1]<<" "<<q_solution[2]<<" "<<q_solution[3]<<" "<<q_solution[4]<<" "<<q_solution[5]<<endl;
            pinocchio::forwardKinematics(*model, *data, q_solution);
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            //limit_joint_changes(q_, q_solution);
            if (output_file_2.is_open()) {
                output_file_2 << "Positions control: "<< data->oMf[tool_frame_id].translation()[0]<<" "<<data->oMf[tool_frame_id].translation()[1]<<" "<<data->oMf[tool_frame_id].translation()[2]<<" ";
                output_file_2 << "Orientation (quaternion): "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).w()<<" "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).x()<<" "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).y()<<" "<< Eigen::Quaterniond(data->oMf[tool_frame_id].rotation()).z()<<endl; 
            }

            // Publicar las nuevas posiciones articulares
            auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
            trajectory_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = {q_solution[0], q_solution[1], q_solution[2], q_solution[3], q_solution[4], q_solution[5]};
            
            point.time_from_start = rclcpp::Duration::from_seconds(ur5_time); // Tiempo para alcanzar la posición
            trajectory_msg.points.push_back(point);
            joint_trajectory_pub_->publish(trajectory_msg);
        }
    };


class JointTrajectoryPublisher : public rclcpp::Node
    {
    public:
    
        JointTrajectoryPublisher() : Node("joint_trajectory_publisher")
        {
            initializeUR5(model, data, tool_frame_id, urdf_path);
            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                control_topic, 10);

            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&JointTrajectoryPublisher::publish_trajectory, this));
        }

    private:
            std::unique_ptr<pinocchio::Model> model; // declarar puntero único para el modelo
            std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos
            pinocchio::FrameIndex tool_frame_id; 
            std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
            std::string urdf_path = get_file_path("ur5_simulation",   "include/ur5.urdf");
            
            rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

        void publish_trajectory()
        {
            // Leer las posiciones articulares desde el archivo config.txt
            std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
            double joint_positions[6];
            cout<<array_length(joint_positions)<<endl;
            try {
                load_values_from_file(config_path, joint_positions, array_length(joint_positions),7); // Leer la 7ma línea
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error al leer las posiciones articulares desde el archivo: %s", e.what());
                return;
            }
            cout<<"joint_positions: "<<joint_positions[0]<<" "<<joint_positions[1]<<" "<<joint_positions[2]<<" "<<joint_positions[3]<<" "<<joint_positions[4]<<" "<<joint_positions[5]<<endl;

            // Crear y publicar el mensaje de trayectoria
            auto message = trajectory_msgs::msg::JointTrajectory();
            message.joint_names = {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {joint_positions[0], joint_positions[1], joint_positions[2],
                            joint_positions[3], joint_positions[4], joint_positions[5]};
            point.time_from_start = rclcpp::Duration::from_seconds(4.0);

            message.points.push_back(point);

            RCLCPP_INFO(this->get_logger(), "Publicando posición articular...");
            publisher_->publish(message);
            pinocchio::forwardKinematics(*model, *data, Eigen::Map<Eigen::VectorXd>(joint_positions, 6));
            pinocchio::updateFramePlacement(*model, *data, tool_frame_id);
            cout<< "x: "<< data->oMf[tool_frame_id].translation()[0]<<" y: "<< data->oMf[tool_frame_id].translation()[1]<<" z: "<< data->oMf[tool_frame_id].translation()[2]<<endl; 
        }

        
        
    };
        


int main(int argc, char **argv) {
    
    // Inicializar ROS2
    int l;int l3;
    cout<<"Simulacion o Implementacion? 1.-Simulacion 2.-Implementacion"<<endl;cin>>l3;
    if (l3==1){
        implementacion = false;
        control_topic = "/joint_trajectory_controller/joint_trajectory";
        cout<<"Elija robot: 1.-UR5 \n 2.-UR5e"<<endl;cin>>l;
        if (l == 1) {
            cout<<"UR5"<<endl;
            ur = "ur5";
        } else if (l == 2) {
            cout<<"UR5e"<<endl;
            ur = "ur5e";
        } else {
            cout<<"Opcion no valida"<<endl;
            return 0;
        }
    }else if (l3 ==2){
        implementacion = true;
        control_topic = "/scaled_joint_trajectory_controller/joint_trajectory";
        cout<<"Elija robot: 1.-UR5 \n 2.-UR5e"<<endl;cin>>l;
        if (l == 1) {
            cout<<"UR5"<<endl;
            ur = "ur5";
        } else if (l == 2) {
            cout<<"UR5e"<<endl;
            ur = "ur5e";
        } else {
            cout<<"Opcion no valida"<<endl;
            return 0;
        }
    }
    
    cout<<"Elija:\n1.-Control con Geomagic\n2.-Trayectoria\n3.-Solo Inversa\n4.-Movimiento Articualar"<<endl;cin>> l;
    if (l == 1) {
        

        int l2;       
        
        cout<<"Con orientacion? 1.-si 2.-no"<<endl;cin>>l2;
        geomagic = true;
        if (l2 == 1) {      orientacion = true;    cout<<"Con orientacion"<<endl;  } 
        else if (l2 == 2) { orientacion = false;   cout<<"Sin orientacion"<<endl;  } 
        else {                                     cout<<"Opcion no valida"<<endl;    return 0; }
        cout<<"Control con Geomagic"<<endl;
        rclcpp::init(argc, argv);
        auto node = std::make_shared<UR5eJointController>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } else if (l == 2) {
        geomagic = false;
        orientacion = true;
        cout<<"Control de Trayectoria"<<endl;
        rclcpp::init(argc, argv);
        auto node = std::make_shared<UR5eJointController>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    } else if (l == 3) {
        cout<<"Solo Inversa"<<endl;
        std::string config_path = get_file_path("ur5_simulation", "include/config.txt");
        std::string urdf_path = get_file_path("ur5_simulation",   "include/"+ur+".urdf");

        double q_init[6];        
        double desired_pose[3];        
        double rot_des[4];
        double max_iteraciones[1];
        double alpha[1];

        double quat[4] = {0,0,1,0};
        Eigen::Matrix3d rot_matrix = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix(); 
        cout<<"rot_matrix: \n"<<rot_matrix<<endl;

        try {
                load_values_from_file(config_path, max_iteraciones, 1, 18);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
            try {
                load_values_from_file(config_path, alpha, 1, 16);       // Leer la 6ta línea para x_init del UR5
            } catch (const std::exception &e) {
                cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;                
            }
        
        try {
            load_values_from_file(config_path, q_init, 6, 1);       // Leer la 1ra línea para q_init
            load_values_from_file(config_path, desired_pose, 3, 2); // Leer la 2da línea para desired_pose
            load_values_from_file(config_path, rot_des, 4, 3);      // Leer la 3ra línea para rot_des
        } catch (const std::exception &e) {
            cerr << "Error al cargar el archivo de configuración: " << e.what() << endl;
            return 1;
        }
        cout<<"q_init: \n"<<q_init[0]<<" "<<q_init[1]<<" "<<q_init[2]<<" "<<q_init[3]<<" "<<q_init[4]<<" "<<q_init[5]<<endl;
        cout<<"desired_pose: \n"<<desired_pose[0]<<" "<<desired_pose[1]<<" "<<desired_pose[2]<<endl;
        cout<<"rot_des: \n"<<rot_des[0]<<" "<<rot_des[1]<<" "<<rot_des[2]<<" "<<rot_des[3]<<endl;

        std::unique_ptr<pinocchio::Model> model;    
        std::unique_ptr<pinocchio::Data> data; // declarar puntero único para los datos
        pinocchio::FrameIndex tool_frame_id;
        initializeUR5(model, data, tool_frame_id, urdf_path);
        Eigen::VectorXd q_result = Cinematica_Inversa(Eigen::Map<Eigen::VectorXd>(q_init, 6), desired_pose,rot_des, max_iteraciones[0], alpha[0], model, data, tool_frame_id);
        cout << "Resultado de la cinemática inversa: " << q_result.transpose() << endl; 
    } else if (l == 4) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<JointTrajectoryPublisher>());
        rclcpp::shutdown();
        return 0;
    }
    
    else {
        cout<<"Opcion no valida"<<endl;
        return 0;
    }
    
}

