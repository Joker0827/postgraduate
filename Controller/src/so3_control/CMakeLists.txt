cmake_minimum_required(VERSION 3.8)
project(so3_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(SO3Control SHARED src/SO3Control.cpp)
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(SO3Control Eigen3::Eigen)

add_executable(so3_control_node src/so3_control_node.cpp)
target_link_libraries(so3_control_node SO3Control)
ament_target_dependencies(so3_control_node rclcpp nav_msgs sensor_msgs std_msgs quadrotor_msgs)

add_executable(network_control_node src/NetworkControl.cpp src/network_control_node.cpp)
target_link_libraries(network_control_node SO3Control)
ament_target_dependencies(network_control_node rclcpp nav_msgs sensor_msgs quadrotor_msgs mavros_msgs)

add_executable(control_example src/control_example.cpp)
ament_target_dependencies(control_example rclcpp quadrotor_msgs)

install(TARGETS SO3Control EXPORT export_SO3Control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS so3_control_node network_control_node control_example
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_SO3Control HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp nav_msgs sensor_msgs std_msgs quadrotor_msgs mavros_msgs)
ament_package()
