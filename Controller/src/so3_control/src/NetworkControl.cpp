#include "so3_control/NetworkControl.h"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <regex>
#include <stdexcept>

NetworkControl::NetworkControl(const rclcpp::NodeOptions &options)
    : rclcpp::Node("network_controller_node", options)
{
    mass_ = declare_parameter("mass", 0.98);
    control_dt_ = declare_parameter("control_dt", 0.02);
    is_simulation_ = declare_parameter("is_simulation", false);
    use_disturbance_observer_ = declare_parameter("use_disturbance_observer", false);
    hover_thrust_ = declare_parameter("hover_thrust", 0.4);
    kx_xy = declare_parameter("kx_xy", 5.7);
    kx_z = declare_parameter("kx_z", 6.2);
    kv_xy = declare_parameter("kv_xy", 3.4);
    kv_z = declare_parameter("kv_z", 4.0);
    record_log_ = declare_parameter("record_log", false);
    logger_file_name = declare_parameter<std::string>("logger_file_name", "/tmp/yopo_logs");
    takeoff_altitude_ = declare_parameter("takeoff_altitude", 2.0);
    transition_timeout_ = declare_parameter("flight_transition_timeout", 8.0);
    auto_takeoff_pending_ = declare_parameter("auto_takeoff", is_simulation_);
    if (!(mass_ > 0.0) || !(control_dt_ > 0.0) || !(transition_timeout_ > 0.0))
        throw std::invalid_argument("mass, control_dt and flight_transition_timeout must be positive");
    so3_controller_.setMass(mass_);
    disturbance_observer_ = HGDO(control_dt_);
    mavros_interface_ = std::make_unique<Mavros_Interface>(*this, is_simulation_);
    so3_command_pub_ = create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);
    position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "position_cmd", rclcpp::QoS(1),
        [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) { network_cmd_callback(msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS().keep_last(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_callback(msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imu_callback(msg); });
    takeoff_land_srv_ = create_service<quadrotor_msgs::srv::SetTakeoffLand>(
        "~/takeoff_land",
        [this](const std::shared_ptr<quadrotor_msgs::srv::SetTakeoffLand::Request> request,
               std::shared_ptr<quadrotor_msgs::srv::SetTakeoffLand::Response> response) {
            response->res = begin_takeoff_land(request->takeoff, request->takeoff_altitude);
        });
    startup_time_ = now();
    phase_started_ = now();
    control_timer_ = rclcpp::create_timer(this, get_clock(),
        rclcpp::Duration::from_seconds(control_dt_), [this]() { timerCallback(); });
    RCLCPP_INFO(get_logger(), "SO3 network control ready; mass=%.3f kg, period=%.3f s",
                mass_, control_dt_);
}

void NetworkControl::initLogRecorder()
{   
    // Use file count as name to avoid date confusion
    std::cout << "logger_file_name: " << logger_file_name << std::endl;
    int max_number = -1;
    std::filesystem::path dir(logger_file_name);
    std::error_code error;
    std::filesystem::create_directories(dir, error);
    if (std::filesystem::exists(dir) && std::filesystem::is_directory(dir)) {
        std::regex filename_pattern(R"(^(\d+)_.*$)"); // 匹配以数字开头，并以"_"分隔的文件名
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (std::filesystem::is_regular_file(entry)) {
                std::string filename = entry.path().filename().string();
                std::smatch match;
                if (std::regex_match(filename, match, filename_pattern)) {
                    int number = std::stoi(match[1]); // 提取第一个"_"前的数字
                    max_number = std::max(max_number, number);
                }
            }
        }
    } else {
        std::cerr << "Error: invalid logger path" << std::endl;
    }
    std::string fileCountStr = std::to_string(max_number + 1);
    std::string temp_file_name = (dir / (fileCountStr + "_Net_logger_")).string();
    time_t timep;
    timep = time(0);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
    temp_file_name += tmp;
    temp_file_name += ".csv";
    if (logger.is_open())
    {
        logger.close();
    }
    logger.open(temp_file_name.c_str(), std::ios::out);
    std::cout << "logger: " << temp_file_name << std::endl;
    if (!logger.is_open())
    {
        std::cout << "cannot open the logger." << std::endl;
    }
    else
    {
        logger << "timestamp" << ',';
        logger << "cur_px" << ',';
        logger << "cur_py" << ',';
        logger << "cur_pz" << ',';
        logger << "cur_vx" << ',';
        logger << "cur_vy" << ',';
        logger << "cur_vz" << ',';
        logger << "cur_ax" << ',';
        logger << "cur_ay" << ',';
        logger << "cur_az" << ',';
        logger << "des_px" << ',';
        logger << "des_py" << ',';
        logger << "des_pz" << ',';
        logger << "des_vx" << ',';
        logger << "des_vy" << ',';
        logger << "des_vz" << ',';
        logger << "des_ax" << ',';
        logger << "des_ay" << ',';
        logger << "des_az" << ',';
        logger << "dis_ax" << ',';
        logger << "dis_ay" << ',';
        logger << "dis_az" << ',';
        logger << "px4_ax" << ',';
        logger << "px4_ay" << ',';
        logger << "px4_az" << ',';
        logger << "thrust" << ',';
        logger << "cur_yaw" << ',';
        logger << "des_yaw" << std::endl;
    }
}

void NetworkControl::recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw)
{
    if (logger.is_open())
    {
        logger << now().nanoseconds() << ',';
        logger << cur_pos_(0) << ',';
        logger << cur_pos_(1) << ',';
        logger << cur_pos_(2) << ',';
        logger << cur_v(0) << ',';
        logger << cur_v(1) << ',';
        logger << cur_v(2) << ',';
        logger << cur_a(0) << ',';
        logger << cur_a(1) << ',';
        logger << cur_a(2) << ',';
        logger << des_pos_(0) << ',';
        logger << des_pos_(1) << ',';
        logger << des_pos_(2) << ',';
        logger << des_vel_(0) << ',';
        logger << des_vel_(1) << ',';
        logger << des_vel_(2) << ',';
        logger << des_a(0) << ',';
        logger << des_a(1) << ',';
        logger << des_a(2) << ',';
        logger << dis_a(0) << ',';
        logger << dis_a(1) << ',';
        logger << dis_a(2) << ',';
        logger << des_a(0) - dis_a(0) << ',';
        logger << des_a(1) - dis_a(1) << ',';
        logger << des_a(2) - dis_a(2) << ',';
        logger << last_thrust_ << ',';
        logger << cur_yaw << ',';
        logger << des_yaw << std::endl;
    }
}

Eigen::Vector3d NetworkControl::publishHoverSO3Command(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, 
                                                       Eigen::Vector3d des_acc, double des_yaw, double des_yaw_dot)
{
    Eigen::Vector3d kx(kx_xy, kx_xy, kx_z);
    Eigen::Vector3d kv(kv_xy, kv_xy, kv_z);
    so3_controller_.calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot, kx, kv);

    Eigen::Vector3d force = so3_controller_.getComputedForce();
    Eigen::Quaterniond orientation = so3_controller_.getComputedOrientation();

    auto so3_command = std::make_shared<quadrotor_msgs::msg::SO3Command>();
    so3_command->header.stamp = now();
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();
    so3_command->k_r[0] = 1.5;
    so3_command->k_r[1] = 1.5;
    so3_command->k_r[2] = 1.0;
    so3_command->k_om[0] = 0.13;
    so3_command->k_om[1] = 0.13;
    so3_command->k_om[2] = 0.1;
    so3_command->aux.current_yaw = cur_yaw_;
    so3_command->aux.enable_motors = true;
    so3_command_pub_->publish(*so3_command);

    double thrust_norm = force.norm() / (mass_ * ONE_G) * hover_thrust_;
    mavros_interface_->pub_att_thrust_cmd(orientation, thrust_norm);
    last_thrust_ = thrust_norm;

    double thrust = force.norm() / mass_;
    Eigen::Matrix3d Cbn;    
    get_dcm_from_q(Cbn, orientation);
    Eigen::Vector3d att_acc = Eigen::Vector3d(0, 0, thrust);
    att_acc = Cbn * att_acc;
    att_acc(2) -= ONE_G;
    // std::cout<<"att_acc"<<att_acc.transpose()<<std::endl;
    return att_acc;
}

Eigen::Vector3d NetworkControl::get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des)
{
    Eigen::Vector3d force_ = mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    force_.noalias() += mass_ * ref_acc;

    // Limit control angle to theta degree
    double theta = M_PI / 4;
    double c = cos(theta);
    Eigen::Vector3d f;
    f.noalias() = force_ - mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
    {
        double nf = f.norm();
        double A = c * c * nf * nf - f(2) * f(2);
        double B = 2 * (c * c - 1) * f(2) * mass_ * ONE_G;
        double C = (c * c - 1) * mass_ * mass_ * ONE_G * ONE_G;
        double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        force_.noalias() = s * f + mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    }

    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(ref_yaw), sin(ref_yaw), 0);

    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    quat_des = Eigen::Quaterniond(R);
    force_des = force_;

    Eigen::Vector3d acc_actual = force_des / mass_ - Eigen::Vector3d(0, 0, ONE_G);
    return acc_actual;
}

// 世界系的期望加速度：ref_acc（加上g）、期望yaw：ref_yaw
Eigen::Vector3d NetworkControl::pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw)
{
    Eigen::Vector3d force;
    Eigen::Quaterniond quat_des;
    Eigen::Vector3d acc_actual = get_Q_from_ACC(ref_acc, ref_yaw, quat_des, force);
    auto so3_command = std::make_shared<quadrotor_msgs::msg::SO3Command>();
    so3_command->header.stamp = now();
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = quat_des.x();
    so3_command->orientation.y = quat_des.y();
    so3_command->orientation.z = quat_des.z();
    so3_command->orientation.w = quat_des.w();
    so3_command->k_r[0] = 1.5;
    so3_command->k_r[1] = 1.5;
    so3_command->k_r[2] = 1.0;
    so3_command->k_om[0] = 0.13;
    so3_command->k_om[1] = 0.13;
    so3_command->k_om[2] = 0.1;
    so3_command->aux.current_yaw = cur_yaw;
    so3_command->aux.enable_motors = true;
    so3_command_pub_->publish(*so3_command);

    double thrust_norm = force.norm() / (mass_ * ONE_G) * hover_thrust_;
    mavros_interface_->pub_att_thrust_cmd(quat_des, thrust_norm);
    last_thrust_ = thrust_norm;
    return acc_actual;
}

void NetworkControl::limite_acc(Eigen::Vector3d &acc){
    return;
    double max_norm = 10.0;
    double norm = acc.norm();
    if (norm > max_norm) {
        acc = acc / norm * max_norm;
    }
}

void NetworkControl::network_cmd_callback(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd)
{
    if (!ctrl_valid_)
        return;

    bool arm_state = false;
    bool ofb_enable = false;
    mavros_interface_->get_status(arm_state, ofb_enable);
    if (!arm_state || !ofb_enable)
        return;

    position_cmd_init_ = true;

    des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    Eigen::Vector3d des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    limite_acc(des_acc);

    double des_yaw = cmd->yaw;
    
    disturbance_observer_.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
    // std::cout << "dis_acc: " << dis_acc_.transpose() << std::endl;

    Eigen::Vector3d att_acc;
    if (cmd->trajectory_flag == quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY)
    {    
        if (use_disturbance_observer_)
            att_acc = des_acc - dis_acc_;
        else
            att_acc = des_acc;
        att_acc = pub_SO3_command(att_acc, des_yaw, cur_yaw_);
        // std::cout<<"acc: "<<des_acc.transpose()<<"   yaw:"<<des_yaw<<std::endl;
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, des_acc, dis_acc_, cur_yaw_, des_yaw);
    }
    else
    {
        double des_yaw_dot = cmd->yaw_dot;
        att_acc = publishHoverSO3Command(des_pos_, des_vel_, des_acc, des_yaw, des_yaw_dot);
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, att_acc, dis_acc_, cur_yaw_, des_yaw);
    }

    last_des_acc_ = att_acc;
}

void NetworkControl::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
    const auto &q = odom->pose.pose.orientation;
    cur_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    cur_vel_ = Eigen::Vector3d(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    cur_pos_ = Eigen::Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    cur_att_.w() = odom->pose.pose.orientation.w;
    cur_att_.x() = odom->pose.pose.orientation.x;
    cur_att_.y() = odom->pose.pose.orientation.y;
    cur_att_.z() = odom->pose.pose.orientation.z;

    // if(!is_simulation_)
    //     cur_acc_ = Eigen::Vector3d(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z);

    so3_controller_.setPosition(cur_pos_);
    so3_controller_.setVelocity(cur_vel_);
    if (!state_init_)
        RCLCPP_INFO(get_logger(), "Odometry received; ready to take off");
    state_init_ = true;
}

void NetworkControl::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
{
    Eigen::Vector3d acc(imu->linear_acceleration.x,
                        imu->linear_acceleration.y,
                        imu->linear_acceleration.z);
    if (is_simulation_)
    {
        cur_acc_ = acc;
    }
    else
    {
        Eigen::Vector3d acc_world = cur_att_ * acc;
        acc_world(2) -= 9.8;
        cur_acc_ = acc_world;
    }

    // so3_controller_.setAcc(acc_world);
}

void NetworkControl::timerCallback()
{
    if (auto_takeoff_pending_ && state_init_ && (now() - startup_time_).seconds() >= 2.0) {
        auto_takeoff_pending_ = false;
        begin_takeoff_land(true, takeoff_altitude_);
    }
    mavros_interface_->poll();
    update_flight_phase();
    if (!state_init_ || !ref_valid_)
        return;
    if (position_cmd_init_ && ctrl_valid_)
        return;

    Eigen::Vector3d att_acc = publishHoverSO3Command(
        des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_);
    if (takeoff_cmd_init_)
        disturbance_observer_.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
    last_des_acc_ = att_acc;
    if (record_log_)
        recordLog(cur_vel_, cur_acc_, att_acc, dis_acc_, cur_yaw_, des_yaw_);
    takeoff_cmd_init_ = true;
}

bool NetworkControl::begin_takeoff_land(bool takeoff, double altitude)
{
    if (!state_init_ || flight_phase_ != FlightPhase::Idle ||
        (takeoff && (!std::isfinite(altitude) || altitude <= 0.0))) {
        RCLCPP_WARN(get_logger(), "Takeoff/landing rejected: no odometry, invalid altitude or transition active");
        return false;
    }
    auto_takeoff_pending_ = false;
    ctrl_valid_ = false;
    position_cmd_init_ = false;
    takeoff_cmd_init_ = false;
    des_pos_ = cur_pos_;
    des_pos_(2) -= 0.2;
    des_vel_.setZero();
    des_acc_.setZero();
    des_yaw_ = cur_yaw_;
    des_yaw_dot_ = 0.0;
    ref_valid_ = true;
    phase_started_ = now();
    if (takeoff) {
        takeoff_altitude_ = altitude;
        mavros_interface_->request_arm_and_offboard();
        flight_phase_ = FlightPhase::WaitingArm;
        RCLCPP_INFO(get_logger(), "Takeoff accepted: target altitude %.2f m", altitude);
    } else {
        flight_phase_ = FlightPhase::Landing;
        RCLCPP_INFO(get_logger(), "Landing accepted");
    }
    return true;
}

void NetworkControl::update_flight_phase()
{
    bool armed, offboard;
    mavros_interface_->get_status(armed, offboard);
    switch (flight_phase_) {
    case FlightPhase::WaitingArm:
        if (mavros_interface_->operation_failed()) {
            flight_phase_ = FlightPhase::Idle;
            ref_valid_ = false;
            RCLCPP_ERROR(get_logger(), "Takeoff aborted: could not arm in OFFBOARD");
        } else if (armed && offboard) {
            if (record_log_)
                initLogRecorder();
            flight_phase_ = FlightPhase::ArmDelay;
            phase_started_ = now();
        }
        break;
    case FlightPhase::ArmDelay:
        if ((now() - phase_started_).seconds() >= 1.0) {
            flight_phase_ = FlightPhase::TakingOff;
            phase_started_ = now();
        }
        break;
    case FlightPhase::TakingOff:
        des_pos_(2) = std::min(takeoff_altitude_, des_pos_(2) + 0.8 * control_dt_);
        if (des_pos_(2) >= takeoff_altitude_) {
            flight_phase_ = FlightPhase::Idle;
            ctrl_valid_ = true;
            RCLCPP_INFO(get_logger(), "Takeoff reference reached; accepting planner commands");
        } else if ((now() - phase_started_).seconds() > transition_timeout_) {
            flight_phase_ = FlightPhase::Idle;
            des_pos_ = cur_pos_;
            RCLCPP_ERROR(get_logger(), "Takeoff timed out; holding current position");
        }
        break;
    case FlightPhase::Landing:
        des_pos_(2) -= 0.4 * control_dt_;
        if (std::fabs(cur_pos_(2)) < 0.1 && std::fabs(cur_vel_(2)) < 1.0) {
            mavros_interface_->request_disarm();
            flight_phase_ = FlightPhase::WaitingDisarm;
        } else if ((now() - phase_started_).seconds() > transition_timeout_) {
            flight_phase_ = FlightPhase::Idle;
            des_pos_ = cur_pos_;
            RCLCPP_ERROR(get_logger(), "Landing timed out; holding current position");
        }
        break;
    case FlightPhase::WaitingDisarm:
        if (mavros_interface_->operation_failed()) {
            flight_phase_ = FlightPhase::Idle;
            des_pos_ = cur_pos_;
            RCLCPP_ERROR(get_logger(), "Disarm failed; holding current position");
        } else if (!armed) {
            flight_phase_ = FlightPhase::Idle;
            ref_valid_ = false;
            if (logger.is_open())
                logger.close();
            RCLCPP_INFO(get_logger(), "Landing complete; vehicle disarmed");
        }
        break;
    case FlightPhase::Idle:
        break;
    }
}
