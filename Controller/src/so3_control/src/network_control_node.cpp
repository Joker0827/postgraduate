#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "so3_control/NetworkControl.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NetworkControl>());
    rclcpp::shutdown();
    return 0;
}
