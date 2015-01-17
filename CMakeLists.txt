cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(mlreg)

# The following line is only needed if you compiled the pcl yourself
#set(PCL_DIR "path/to/your/local/pcl/build")
find_package(PCL 1.7 REQUIRED)

# Activate c++11 features
SET(CMAKE_CXX_FLAGS "-std=c++0x")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (mlreg mlreg.cpp)
add_executable (vis_single_pointcloud vis_single_pointcloud.cpp)
add_executable (vis_digest_match vis_digest_match.cpp)
target_link_libraries (mlreg ${PCL_LIBRARIES})
target_link_libraries (vis_single_pointcloud ${PCL_LIBRARIES})
target_link_libraries (vis_digest_match ${PCL_LIBRARIES})
