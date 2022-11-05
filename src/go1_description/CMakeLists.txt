cmake_minimum_required(VERSION 3.8)
project(go1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  xacro
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()