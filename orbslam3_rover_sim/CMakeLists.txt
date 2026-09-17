cmake_minimum_required(VERSION 3.8)
project(orbslam3_rover_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models urdf config rviz
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/auto_loop.py scripts/key_teleop.py scripts/align_frames.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
