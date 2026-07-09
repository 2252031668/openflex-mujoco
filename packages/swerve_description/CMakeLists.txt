cmake_minimum_required(VERSION 3.8)
project(swerve_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
