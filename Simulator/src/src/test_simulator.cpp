#include "sensor_simulator.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SensorSimulator>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("sensor_simulator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
