#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ur5_kinematics/kinematics.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ctime>

struct PinocchioResourcesSimple {
    std::unique_ptr<pinocchio::Model> model;
    std::unique_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex tool_frame_id{0};
};

class ManualIKNode : public rclcpp::Node {
public:
    ManualIKNode() : Node("manual_ik_node") {
        // Parámetros básicos
        this->declare_parameter<std::string>("control_topic", "/scaled_joint_trajectory_controller/joint_trajectory");
        this->declare_parameter<std::string>("nmspace", "");
        this->declare_parameter<std::string>("ur", "ur5");
        this->declare_parameter<std::string>("urdf_path", "");
        this->declare_parameter<std::string>("cartesian_goal_topic", "/manual_cartesian_goal");
        this->declare_parameter<double>("publish_time", 0.05); // segundos para time_from_start
        // Parámetros CSV
        this->declare_parameter<bool>("csv_log_enable", false);
        this->declare_parameter<std::string>("csv_log_dir", "");
        this->declare_parameter<std::string>("csv_log_prefix", "manual_ik");

        this->get_parameter("control_topic", control_topic_);
        this->get_parameter("nmspace", nmspace_);
        this->get_parameter("ur", ur_model_);
        this->get_parameter("urdf_path", urdf_param_);
        this->get_parameter("cartesian_goal_topic", cartesian_goal_topic_);
        this->get_parameter("publish_time", publish_time_);
        this->get_parameter("csv_log_enable", csv_enabled_);
        this->get_parameter("csv_log_dir", csv_path_);
        this->get_parameter("csv_log_prefix", csv_prefix_);

        if (urdf_param_.empty()) {
            std::string share = ament_index_cpp::get_package_share_directory("ur5_description");
            urdf_path_ = share + "/urdf/" + ur_model_ + ".urdf";
        } else {
            urdf_path_ = urdf_param_;
        }
        RCLCPP_INFO(this->get_logger(), "Usando URDF: %s", urdf_path_.c_str());

        initializePinocchio();
        kinematics_solver_ = std::make_unique<UR5Kinematics>(urdf_path_);

        std::string control_topic_ns = "/" + nmspace_ + control_topic_;
        std::string joint_states_topic = nmspace_.empty() ? std::string("/joint_states") : std::string("/") + nmspace_ + std::string("/joint_states");

        joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(control_topic_ns, 10);
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&ManualIKNode::jointStatesCb, this, std::placeholders::_1));
        cartesian_goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(cartesian_goal_topic_, 10, std::bind(&ManualIKNode::goalCb, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Suscrito a joint_states: %s", joint_states_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Suscrito a cartesian_goal: %s", cartesian_goal_topic_.c_str());

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ManualIKNode::tick, this));
        if (csv_enabled_) initCsvLogger();
    }

private:
    // Parámetros
    std::string control_topic_{};
    std::string nmspace_{};
    std::string ur_model_{};
    std::string urdf_param_{};
    std::string urdf_path_{};
    std::string cartesian_goal_topic_{};
    double publish_time_{0.05};
    bool csv_enabled_{false};
    std::string csv_path_{};
    std::string csv_prefix_{"manual_ik"};
    std::ofstream csv_file_{};
    rclcpp::Time start_time_{};
    rclcpp::Time last_log_time_{};
    double last_ik_ms_{0.0};
    double last_loop_ms_{0.0};
    Eigen::VectorXd q_solution_cached_{Eigen::VectorXd::Zero(6)};

    // ROS interfaces
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cartesian_goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Estado
    sensor_msgs::msg::JointState::SharedPtr last_js_;
    Eigen::VectorXd q_current_{Eigen::VectorXd::Zero(6)};
    bool joint_map_initialized_{false};
    std::vector<int> joint_index_map_{std::vector<int>(6,-1)};
    std::vector<std::string> last_js_names_{};
    Eigen::Vector3d cart_goal_{Eigen::Vector3d::Zero()};
    bool cart_goal_received_{false};
    bool initial_pose_captured_{false};
    Eigen::Quaterniond orientation_initial_{Eigen::Quaterniond::Identity()};

    // IK
    std::unique_ptr<UR5Kinematics> kinematics_solver_;
    PinocchioResourcesSimple pinocchio_{};

    void initializePinocchio() {
        pinocchio_.model = std::make_unique<pinocchio::Model>();
        try {
            pinocchio::urdf::buildModel(urdf_path_, *pinocchio_.model);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Error cargando URDF: %s", e.what());
            throw;
        }
        pinocchio_.data = std::make_unique<pinocchio::Data>(*pinocchio_.model);
        pinocchio_.tool_frame_id = pinocchio_.model->getFrameId("tool0");
        if (pinocchio_.tool_frame_id == pinocchio_.model->nframes) {
            throw std::runtime_error("Frame tool0 no encontrado");
        }
    }

    static std::string getHomeDir(){ const char* h=getenv("HOME"); return h?std::string(h):std::string("."); }
    std::string nowTimestampString(){ auto now=std::chrono::system_clock::now(); std::time_t tt=std::chrono::system_clock::to_time_t(now); std::tm tm{}; localtime_r(&tt,&tm); std::ostringstream oss; oss<<std::put_time(&tm,"%Y%m%d_%H%M%S"); return oss.str(); }
    void initCsvLogger(){
        if (csv_path_.empty()) csv_path_ = getHomeDir() + std::string("/.ros/ur5_logs");
        std::error_code ec; std::filesystem::create_directories(csv_path_, ec);
        if (ec) RCLCPP_WARN(this->get_logger(), "No se pudo crear '%s': %s", csv_path_.c_str(), ec.message().c_str());
        std::string ns_part = nmspace_.empty()?"nonamespace":nmspace_;
        std::string fname = csv_prefix_ + std::string("_") + ns_part + std::string("_") + nowTimestampString() + std::string(".csv");
        auto full = (std::filesystem::path(csv_path_) / fname).string();
        csv_file_.open(full, std::ios::out);
        if(!csv_file_.is_open()){ csv_enabled_ = false; RCLCPP_ERROR(this->get_logger(), "No se abre CSV: %s", full.c_str()); return; }
        start_time_ = this->now(); last_log_time_ = start_time_;
        csv_file_ << "t,dt,q_meas_0,q_meas_1,q_meas_2,q_meas_3,q_meas_4,q_meas_5"
                  << ",q_cmd_0,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5"
                  << ",e_q_0,e_q_1,e_q_2,e_q_3,e_q_4,e_q_5"
                  << ",x_des_x,x_des_y,x_des_z,x_meas_x,x_meas_y,x_meas_z"
                  << ",q_des_w,q_des_x,q_des_y,q_des_z,q_meas_w,q_meas_x,q_meas_y,q_meas_z"
                  << ",euler_des_r,euler_des_p,euler_des_y,euler_meas_r,euler_meas_p,euler_meas_y"
                  << ",e_R_angle,pos_err_x,pos_err_y,pos_err_z"
                  << ",ori_err_axis_x,ori_err_axis_y,ori_err_axis_z,ori_err_angle"
                  << ",ik_ms,loop_ms" << std::endl;
        RCLCPP_INFO(this->get_logger(), "CSV habilitado: %s", full.c_str());
    }
    void writeCsvRow(double t,double dt,const Eigen::Vector3d& x_des,const Eigen::Vector3d& x_meas,const Eigen::Quaterniond& q_des,const Eigen::Quaterniond& q_meas,const Eigen::Vector3d& euler_des,const Eigen::Vector3d& euler_meas,double e_R_angle,const Eigen::Vector3d& pos_err,const Eigen::Vector3d& ori_axis,double ori_angle){
        if(!csv_enabled_ || !csv_file_.is_open()) return;
        csv_file_<<std::fixed<<std::setprecision(6)<<t<<","<<dt;
        for(int i=0;i<6;++i) csv_file_<<","<<q_current_[i];
        for(int i=0;i<6;++i) csv_file_<<","<<q_solution_cached_[i];
        for(int i=0;i<6;++i) csv_file_<<","<<(q_solution_cached_[i]-q_current_[i]);
        csv_file_<<","<<x_des.x()<<","<<x_des.y()<<","<<x_des.z()<<","<<x_meas.x()<<","<<x_meas.y()<<","<<x_meas.z();
        csv_file_<<","<<q_des.w()<<","<<q_des.x()<<","<<q_des.y()<<","<<q_des.z()<<","<<q_meas.w()<<","<<q_meas.x()<<","<<q_meas.y()<<","<<q_meas.z();
        csv_file_<<","<<euler_des.x()<<","<<euler_des.y()<<","<<euler_des.z()<<","<<euler_meas.x()<<","<<euler_meas.y()<<","<<euler_meas.z();
        csv_file_<<","<<e_R_angle<<","<<pos_err.x()<<","<<pos_err.y()<<","<<pos_err.z();
        csv_file_<<","<<ori_axis.x()<<","<<ori_axis.y()<<","<<ori_axis.z()<<","<<ori_angle;
        csv_file_<<","<<last_ik_ms_<<","<<last_loop_ms_<<std::endl;
    }

    std::vector<std::string> expectedJointNames() const {
        std::string prefix = nmspace_.empty() ? std::string("") : nmspace_ + std::string("_");
        return {prefix+"shoulder_pan_joint", prefix+"shoulder_lift_joint", prefix+"elbow_joint", prefix+"wrist_1_joint", prefix+"wrist_2_joint", prefix+"wrist_3_joint"};
    }
    std::vector<std::string> expectedBaseJointNames() const {
        return {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    }
    static bool endsWith(const std::string& s, const std::string& suf){
        if (suf.size()>s.size()) return false; return std::equal(suf.rbegin(), suf.rend(), s.rbegin());
    }
    bool sameList(const std::vector<std::string>& a,const std::vector<std::string>& b){
        if (a.size()!=b.size()) return false; for(size_t i=0;i<a.size();++i) if(a[i]!=b[i]) return false; return true;
    }

    void rebuildJointMap(const sensor_msgs::msg::JointState::SharedPtr& msg){
        joint_map_initialized_ = false;
        joint_index_map_.assign(6,-1);
        last_js_names_ = msg->name;
        auto expected = expectedJointNames();
        auto expected_base = expectedBaseJointNames();
        std::unordered_map<std::string,int> name_to_idx;
        for(size_t i=0;i<msg->name.size();++i) name_to_idx[msg->name[i]] = (int)i;
        int found=0;
        for(int i=0;i<6;++i){ auto it=name_to_idx.find(expected[i]); if(it!=name_to_idx.end()){joint_index_map_[i]=it->second; ++found;} }
        if(found<6){
            for(int i=0;i<6;++i){ if(joint_index_map_[i]!=-1) continue; auto it=name_to_idx.find(expected_base[i]); if(it!=name_to_idx.end()){ joint_index_map_[i]=it->second; ++found; } }
        }
        if(found<6){
            for(int i=0;i<6;++i){ if(joint_index_map_[i]!=-1) continue; for(size_t j=0;j<msg->name.size();++j){ if(endsWith(msg->name[j], expected_base[i])){ joint_index_map_[i]=(int)j; ++found; break; } } }
        }
        if(found==6){ joint_map_initialized_=true; RCLCPP_INFO(this->get_logger(), "Mapa de joints listo"); }
        else { RCLCPP_WARN(this->get_logger(), "No se pudo mapear joints (encontrados %d)", found); }
    }

    void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg){
        last_js_ = msg;
        if(!joint_map_initialized_ || !sameList(msg->name, last_js_names_)) rebuildJointMap(msg);
        if(!joint_map_initialized_) return;
        for(int i=0;i<6;++i){ int j = joint_index_map_[i]; if(j<0 || (size_t)j>=msg->position.size()) return; q_current_[i] = msg->position[j]; }
        if(!initial_pose_captured_){
            pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, q_current_);
            pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
            const auto& frame = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
            orientation_initial_ = Eigen::Quaterniond(frame.rotation());
            initial_pose_captured_ = true;
            RCLCPP_INFO(this->get_logger(), "Orientación inicial capturada.");
        }
    }

    void goalCb(const geometry_msgs::msg::PointStamped::SharedPtr msg){
        if(!msg) return;
        cart_goal_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
        cart_goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Nuevo objetivo cartesiano: (%.3f, %.3f, %.3f)", cart_goal_.x(), cart_goal_.y(), cart_goal_.z());
    }

    void tick(){
        auto loop_t0 = std::chrono::steady_clock::now();
        if(!initial_pose_captured_ || !cart_goal_received_ || !joint_map_initialized_) return;
        // FK actual
        pinocchio::forwardKinematics(*pinocchio_.model, *pinocchio_.data, q_current_);
        pinocchio::updateFramePlacement(*pinocchio_.model, *pinocchio_.data, pinocchio_.tool_frame_id);
        const auto& frame_meas = pinocchio_.data->oMf[pinocchio_.tool_frame_id];
        Eigen::Vector3d x_meas = frame_meas.translation();
        Eigen::Matrix3d R_meas = frame_meas.rotation();
        Eigen::Quaterniond q_meas(R_meas);
        Eigen::Vector3d euler_meas = R_meas.eulerAngles(0,1,2);
        // Objetivos
        Eigen::Vector3d x_des = cart_goal_;
        Eigen::Quaterniond q_des = orientation_initial_;
        Eigen::Matrix3d R_des = q_des.toRotationMatrix();
        Eigen::Matrix3d R_err = R_meas.transpose() * R_des;
        double e_R_angle = Eigen::AngleAxisd(R_err).angle();
        Eigen::Vector3d euler_des = R_des.eulerAngles(0,1,2);
        // IK
        auto ik_t0 = std::chrono::steady_clock::now();
        q_solution_cached_ = kinematics_solver_->inverseKinematicsQP(q_current_, x_des, q_des, 600, 0.1);
        last_ik_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-ik_t0).count()/1000.0;
        std::cout<<"tiempo de IK: "<<last_ik_ms_<<" ms"<<std::endl;
        publishTrajectory(q_solution_cached_);
        last_loop_ms_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-loop_t0).count()/1000.0;
        if(csv_enabled_){
            rclcpp::Time now_ros = this->now();
            double t = (now_ros - start_time_).seconds();
            double dt = (now_ros - last_log_time_).seconds();
            last_log_time_ = now_ros;
            Eigen::Vector3d pos_err = x_des - x_meas;
            Eigen::Quaterniond q_err = q_des * q_meas.inverse();
            Eigen::AngleAxisd aa(q_err);
            Eigen::Vector3d ori_axis = aa.axis();
            double ori_angle = aa.angle();
            writeCsvRow(t, dt, x_des, x_meas, q_des, q_meas, euler_des, euler_meas, e_R_angle, pos_err, ori_axis, ori_angle);
        }
    }

    void publishTrajectory(const Eigen::VectorXd& q){
        std::string prefix = nmspace_.empty() ? std::string("") : nmspace_ + std::string("_");
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {prefix+"shoulder_pan_joint", prefix+"shoulder_lift_joint", prefix+"elbow_joint", prefix+"wrist_1_joint", prefix+"wrist_2_joint", prefix+"wrist_3_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions.assign(q.data(), q.data()+q.size());
        pt.time_from_start = rclcpp::Duration::from_seconds(publish_time_);
        traj.points.push_back(pt);
        joint_traj_pub_->publish(traj);
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualIKNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
