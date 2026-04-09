cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
