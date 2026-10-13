cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf meshes licenses
  DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)

ament_package()
