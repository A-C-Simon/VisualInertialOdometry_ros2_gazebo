cmake_minimum_required(VERSION 3.8)
project(ov_hw290)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

add_executable(stereo_splitter src/stereo_splitter.cpp)
ament_target_dependencies(stereo_splitter rclcpp sensor_msgs)
target_include_directories(stereo_splitter PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_splitter ${OpenCV_LIBS})
target_compile_features(stereo_splitter PUBLIC cxx_std_17)
target_compile_options(stereo_splitter PRIVATE -O2 -Wall -Wextra)

install(TARGETS stereo_splitter DESTINATION lib/${PROJECT_NAME})
ament_package()
