cmake_minimum_required(VERSION 3.8)
project(robot_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config worlds maps models licenses DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/generate_demo_world.py scripts/smoke_test.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
