cmake_minimum_required(VERSION 3.16)
project(so3_quadrotor_simulator)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
target_compile_features(quadrotor_dynamics PUBLIC cxx_std_17)
target_include_directories(quadrotor_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ode>
  $<INSTALL_INTERFACE:include> ${Boost_INCLUDE_DIRS})
target_link_libraries(quadrotor_dynamics PUBLIC Eigen3::Eigen)

add_executable(quadrotor_simulator_so3 src/quadrotor_simulator_so3.cpp)
target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)
ament_target_dependencies(quadrotor_simulator_so3
  rclcpp quadrotor_msgs uav_utils nav_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros)
install(TARGETS quadrotor_dynamics EXPORT quadrotor_dynamics
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS quadrotor_simulator_so3 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/quadrotor_simulator DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_export_targets(quadrotor_dynamics HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost)
ament_package()
