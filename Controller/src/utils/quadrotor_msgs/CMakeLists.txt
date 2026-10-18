cmake_minimum_required(VERSION 3.16)
project(quadrotor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/LQRTrajectory.msg"
  "srv/SetTakeoffLand.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(quadrotor_serial_codec src/encode_msgs.cpp src/decode_msgs.cpp)
target_compile_features(quadrotor_serial_codec PUBLIC cxx_std_17)
target_include_directories(quadrotor_serial_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(quadrotor_serial_codec PUBLIC "${cpp_typesupport_target}" Eigen3::Eigen)
install(TARGETS quadrotor_serial_codec EXPORT quadrotor_serial_codec
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(quadrotor_serial_codec HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime Eigen3)
ament_package()
