cmake_minimum_required(VERSION 3.16)
project(yopo)
find_package(ament_cmake REQUIRED)

install(PROGRAMS test_yopo_ros.py train_yopo.py yopo_trt_transfer.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES ros_utils.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config policy loss saved DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "events.out.*" EXCLUDE)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES yopo.rviz DESTINATION share/${PROJECT_NAME})
ament_package()
