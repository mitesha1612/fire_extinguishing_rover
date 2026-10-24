cmake_minimum_required(VERSION 3.8)
project(fire_rover_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
    worlds
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
