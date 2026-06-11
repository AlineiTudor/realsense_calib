cmake_minimum_required(VERSION 3.8)
project(multicam_calibration)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  multicam_calibration/generate_boards.py
  multicam_calibration/calibration_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
