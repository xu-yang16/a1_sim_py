cmake_minimum_required(VERSION 3.0.2)
project(quadruped_unitree)
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()