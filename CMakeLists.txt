cmake_minimum_required(VERSION 3.20)
project(depth_cloud_point)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK 7 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(cloud_by_pcl)
add_subdirectory(cloud_by_pangolin)

add_library(GetCamera SHARED get_camera/get_camera_parameter.cpp get_camera/get_camera_parameter.h )
target_link_libraries(GetCamera ${OpenCV_LIBS} -ldl)


include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

add_executable(depth_cloud_point main.cpp)

target_link_libraries (depth_cloud_point ${VTK_LIBRARIES} ${PCL_LIBRARIES})