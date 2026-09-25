cmake_minimum_required(VERSION 3.8)
project(orb_slam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# The ORB-SLAM3 project does not install a CMake package. Point this at the
# already-built source tree (or export ORB_SLAM3_ROOT before invoking colcon).
set(ORB_SLAM3_ROOT "$ENV{ORB_SLAM3_ROOT}" CACHE PATH "Built ORB-SLAM3 source tree")
if(NOT ORB_SLAM3_ROOT)
  if(DEFINED ENV{HOME})
    set(ORB_SLAM3_ROOT "$ENV{HOME}/ORB_SLAM3")
  endif()
endif()

find_path(ORB_SLAM3_INCLUDE_DIR System.h
  PATHS "${ORB_SLAM3_ROOT}/include"
  NO_DEFAULT_PATH)
find_library(ORB_SLAM3_LIBRARY ORB_SLAM3
  PATHS "${ORB_SLAM3_ROOT}/lib"
  NO_DEFAULT_PATH)
find_library(DBOW2_LIBRARY DBoW2
  PATHS "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib"
  NO_DEFAULT_PATH)
find_library(G2O_LIBRARY g2o
  PATHS "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  NO_DEFAULT_PATH)

if(NOT ORB_SLAM3_INCLUDE_DIR OR NOT ORB_SLAM3_LIBRARY OR
   NOT DBOW2_LIBRARY OR NOT G2O_LIBRARY OR
   NOT EXISTS "${ORB_SLAM3_ROOT}/Thirdparty/Sophus/sophus/se3.hpp")
  message(FATAL_ERROR
    "A built ORB-SLAM3 tree was not found at '${ORB_SLAM3_ROOT}'. "
    "Build ORB-SLAM3 first, then use: colcon build --cmake-args "
    "-DORB_SLAM3_ROOT=/absolute/path/to/ORB_SLAM3")
endif()

add_executable(stereo_node src/stereo_node.cpp)
target_include_directories(stereo_node SYSTEM PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_INCLUDE_DIR}"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(stereo_node
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros)
target_link_libraries(stereo_node
  "${ORB_SLAM3_LIBRARY}"
  "${DBOW2_LIBRARY}"
  "${G2O_LIBRARY}"
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(stereo_imu_node src/stereo_imu_node.cpp)
target_include_directories(stereo_imu_node SYSTEM PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_INCLUDE_DIR}"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(stereo_imu_node
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros)
target_link_libraries(stereo_imu_node
  "${ORB_SLAM3_LIBRARY}"
  "${DBOW2_LIBRARY}"
  "${G2O_LIBRARY}"
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS})

# Keep the executable usable from both build/ and install/ without modifying
# LD_LIBRARY_PATH. These libraries remain in the external ORB-SLAM3 tree.
set_target_properties(stereo_node stereo_imu_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")

install(TARGETS stereo_node stereo_imu_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
