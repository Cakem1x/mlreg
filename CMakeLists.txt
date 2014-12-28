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

add_executable (mlreg mlreg.cpp Digest.hpp)
target_link_libraries (mlreg ${PCL_LIBRARIES})
