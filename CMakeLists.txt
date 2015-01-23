cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(mlreg)

# The following line is only needed if you compiled the pcl yourself
#set(PCL_DIR "path/to/your/local/pcl/build")
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

# Activate c++11 features
SET(CMAKE_CXX_FLAGS "-std=c++0x")

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${OPENCV_DEFINITIONS})

#add_executable (mlreg mlreg.cpp)
add_executable (vis_single_pointcloud vis_single_pointcloud.cpp)
add_executable (vis_digest_match vis_digest_match.cpp)
#target_link_libraries (mlreg ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (vis_single_pointcloud ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries (vis_digest_match ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
