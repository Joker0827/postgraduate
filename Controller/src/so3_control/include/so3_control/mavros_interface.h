#ifndef MAVROS_INTERFACE_H_
#define MAVROS_INTERFACE_H_

#include <chrono>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

class Mavros_Interface
{
public:
    explicit Mavros_Interface(rclcpp::Node &node, bool simulation)
        : node_(node), simulation_(simulation)
    {
        const auto base = node_.declare_parameter<std::string>("mavros_namespace", "/mavros");
        att_target_pub_ = node_.create_publisher<mavros_msgs::msg::AttitudeTarget>(
            base + "/setpoint_raw/attitude", 10);
        if (!simulation_) {
            state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
                base + "/state", rclcpp::SensorDataQoS(),
                [this](mavros_msgs::msg::State::ConstSharedPtr state) {
                    has_armed_ = state->armed;
                    offboard_enabled_ = state->mode == "OFFBOARD";
                });
        }
        set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(base + "/set_mode");
        arm_disarm_client_ = node_.create_client<mavros_msgs::srv::CommandBool>(base + "/cmd/arming");
    }

    void get_status(bool &armed, bool &offboard) const
    {
        armed = has_armed_;
        offboard = offboard_enabled_;
    }

    void request_arm_and_offboard() { begin_operation(true); }
    void request_disarm() { begin_operation(false); }
    bool operation_failed() const { return failed_; }
    bool operation_pending() const { return active_; }

    // All callbacks and poll execute in the node's default callback group.
    void poll()
    {
        if (!active_)
            return;
        if ((arming_ && has_armed_ && offboard_enabled_) || (!arming_ && !has_armed_)) {
            cancel_pending_request();
            active_ = false;
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - started_ > std::chrono::seconds(15)) {
            fail("Timed out waiting for MAVROS state confirmation");
            return;
        }
        if (pending_) {
            if (now - last_request_ > std::chrono::seconds(3)) {
                cancel_pending_request();
            }
            return;
        }
        if (now - last_request_ < std::chrono::seconds(1))
            return;
        const bool set_mode = arming_ && !offboard_enabled_;
        if (set_mode != request_is_mode_) {
            request_is_mode_ = set_mode;
            attempts_ = 0;
        }
        if (attempts_ >= 3) {
            fail("MAVROS operation failed after 3 attempts");
            return;
        }
        if (set_mode) {
            if (!set_mode_client_->service_is_ready())
                return;
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->base_mode = 0;
            request->custom_mode = "OFFBOARD";
            auto future = set_mode_client_->async_send_request(request,
                [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture result) {
                    pending_ = false;
                    if (!result.get()->mode_sent)
                        RCLCPP_WARN(node_.get_logger(), "MAVROS rejected OFFBOARD mode request");
                });
            request_id_ = future.request_id;
        } else {
            if (!arm_disarm_client_->service_is_ready())
                return;
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = arming_;
            auto future = arm_disarm_client_->async_send_request(request,
                [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture result) {
                    pending_ = false;
                    if (!result.get()->success)
                        RCLCPP_WARN(node_.get_logger(), "MAVROS rejected arm/disarm request");
                });
            request_id_ = future.request_id;
        }
        pending_ = true;
        ++attempts_;
        last_request_ = now;
    }

    void pub_att_thrust_cmd(const Eigen::Quaterniond &q, double thrust)
    {
        mavros_msgs::msg::AttitudeTarget command;
        command.header.stamp = node_.now();
        command.type_mask = command.IGNORE_ROLL_RATE | command.IGNORE_PITCH_RATE |
                            command.IGNORE_YAW_RATE;
        command.thrust = static_cast<float>(thrust);
        command.orientation.w = q.w();
        command.orientation.x = q.x();
        command.orientation.y = -q.y();
        command.orientation.z = -q.z();
        att_target_pub_->publish(command);
    }

private:
    void begin_operation(bool arming)
    {
        cancel_pending_request();
        arming_ = arming;
        failed_ = false;
        attempts_ = 0;
        request_is_mode_ = arming;
        started_ = std::chrono::steady_clock::now();
        last_request_ = started_ - std::chrono::seconds(1);
        if (simulation_) {
            has_armed_ = arming;
            offboard_enabled_ = arming;
            active_ = false;
        } else {
            active_ = true;
        }
    }

    void fail(const char *message)
    {
        active_ = false;
        failed_ = true;
        cancel_pending_request();
        RCLCPP_ERROR(node_.get_logger(), "%s", message);
    }

    void cancel_pending_request()
    {
        if (!pending_)
            return;
        if (request_is_mode_)
            set_mode_client_->remove_pending_request(request_id_);
        else
            arm_disarm_client_->remove_pending_request(request_id_);
        pending_ = false;
    }

    rclcpp::Node &node_;
    bool simulation_;
    bool has_armed_{false}, offboard_enabled_{false};
    bool active_{false}, arming_{false}, failed_{false}, pending_{false};
    bool request_is_mode_{false};
    int attempts_{0};
    int64_t request_id_{0};
    std::chrono::steady_clock::time_point started_, last_request_;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_target_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_disarm_client_;
};

#endif
