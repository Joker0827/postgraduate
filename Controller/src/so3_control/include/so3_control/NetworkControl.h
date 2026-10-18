#ifndef NETWORK_CONTROL_H_
#define NETWORK_CONTROL_H_

#include <Eigen/Eigen>
#include <fstream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_msgs/srv/set_takeoff_land.hpp>
#include <so3_control/SO3Control.h>
#include <so3_control/HGDO.h>
#include <so3_control/mavros_interface.h>

#define ONE_G 9.81

class NetworkControl : public rclcpp::Node
{
public:
    explicit NetworkControl(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    enum class FlightPhase { Idle, WaitingArm, ArmDelay, TakingOff, Landing, WaitingDisarm };
    rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Service<quadrotor_msgs::srv::SetTakeoffLand>::SharedPtr takeoff_land_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    double mass_{0.98}, control_dt_{0.02}, hover_thrust_{0.4};
    double kx_xy, kx_z, kv_xy, kv_z;
    double cur_yaw_{0};
    Eigen::Vector3d cur_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d cur_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d cur_acc_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond cur_att_{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d dis_acc_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_des_acc_{Eigen::Vector3d::Zero()};
    double last_thrust_{0};
    Eigen::Vector3d des_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d des_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d des_acc_{Eigen::Vector3d::Zero()};
    double des_yaw_{0}, des_yaw_dot_{0};
    bool is_simulation_{false}, state_init_{false}, ref_valid_{false}, ctrl_valid_{false};
    bool position_cmd_init_{false}, takeoff_cmd_init_{false};
    bool use_disturbance_observer_{false}, record_log_{false}, auto_takeoff_pending_{false};
    double takeoff_altitude_{2.0}, transition_timeout_{8.0};
    FlightPhase flight_phase_{FlightPhase::Idle};
    rclcpp::Time phase_started_, startup_time_;
    SO3Control so3_controller_;
    HGDO disturbance_observer_;
    std::unique_ptr<Mavros_Interface> mavros_interface_;
    std::ofstream logger;
    std::string logger_file_name;

    void initLogRecorder();
    void recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a,
                   Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw);
    Eigen::Vector3d publishHoverSO3Command(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel,
        Eigen::Vector3d des_acc, double des_yaw, double des_yaw_dot);
    Eigen::Vector3d get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw,
        Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des);
    Eigen::Vector3d pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw);
    void limite_acc(Eigen::Vector3d &acc);
    void network_cmd_callback(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd);
    void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom);
    void imu_callback(sensor_msgs::msg::Imu::ConstSharedPtr imu);
    void timerCallback();
    bool begin_takeoff_land(bool takeoff, double altitude);
    void update_flight_phase();
};

#endif
