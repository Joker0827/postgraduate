#include <chrono>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("quad_sim_example");
  auto publisher = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 10);
  int sample = 0;
  auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]() {
    quadrotor_msgs::msg::PositionCommand command;
    command.header.stamp = node->now();
    const int stage = (sample / 500) % 3;
    if (stage == 0) {
      command.position.x = 2.0;
      command.position.z = 1.0;
    } else {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      command.position.x = nan;
      command.position.y = nan;
      command.position.z = nan;
      if (stage == 1) {
        command.velocity.x = -1.0;
      } else {
        command.velocity.x = nan;
        command.velocity.y = nan;
        command.velocity.z = nan;
        command.acceleration.x = 1.0;
      }
    }
    if (sample % 500 == 0)
      RCLCPP_INFO(node->get_logger(), "Control example stage %d", stage + 1);
    publisher->publish(command);
    sample = (sample + 1) % 1500;
  });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
