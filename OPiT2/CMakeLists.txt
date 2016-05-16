#check for the mininum version of cmake
cmake_minimum_required (VERSION 3.0 FATAL_ERROR)

#project name
project (OPiT2)

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#pcl
find_package (PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#compile flag
add_definitions(-DLOGMODE)
# add_definitions(-DKITTI_DATASET)

#set compiler flag to C++11
set (CMAKE_CXX_COMPILER "/usr/bin/clang++")
set (CMAKE_CXX_FLAGS    "-std=c++11 -stdlib=libc++ -Os")

#add subdirectory
add_subdirectory (shared)
add_subdirectory (exec)
#provide main dedicated for testing pcl
add_subdirectory (tests-pcl)
#provide main dedicated for testing visual odometry
add_subdirectory (tests-vo)
#provide main dedicated for whole simulation: vo, camera pose, reprojection, pcl
add_subdirectory (tests-simulation)
