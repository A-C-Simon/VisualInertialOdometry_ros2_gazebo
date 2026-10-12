cmake_minimum_required(VERSION 3.16)
project(orb_online_benchmark LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(ORB_SLAM3_ROOT "/home/ac/ORB_SLAM3" CACHE PATH "Built ORB-SLAM3 tree")
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
add_executable(euroc_online euroc_online.cc)
target_compile_definitions(euroc_online PRIVATE COMPILEDWITHC11)
target_include_directories(euroc_online PRIVATE
 "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include" "${ORB_SLAM3_ROOT}/include/CameraModels"
 "${ORB_SLAM3_ROOT}/Thirdparty/Sophus" ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(euroc_online PRIVATE "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so" ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
