cmake_minimum_required(VERSION 3.20)
project(pcl_point_cloud)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_point_cloud pcl_cloud_point.cpp)

target_link_libraries (pcl_point_cloud ${PCL_LIBRARIES} ${OpenCV_LIBS})

