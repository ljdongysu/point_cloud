cmake_minimum_required(VERSION 3.20)
project(pangolin_point_cloud)

find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)


set(CMAKE_CXX_STANDARD 14)

include_directories(../utils)

include_directories(${Pangolin_LIBRARIES})
link_directories(${Pangolin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message("aaa: " ${Pangolin_LIBRARY_DIRS})
message("bbb: " ${Pangolin_LIBRARIES})
include_directories(../)
add_executable(pangolin_point_cloud disparity.cpp)

target_link_libraries (pangolin_point_cloud ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES} GetCamera)

