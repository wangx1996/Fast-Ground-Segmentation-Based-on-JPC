cmake_minimum_required(VERSION 3.0.2)
project(jpc_seg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)

add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pub_lidar
   CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
  ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


add_executable(jpc_seg_node src/kitti_topic_main.cpp src/jpc_groundremove.cpp)
target_link_libraries(jpc_seg_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_executable(jpc_seg_node2 src/kitti_file_main.cpp src/jpc_groundremove.cpp)
target_link_libraries(jpc_seg_node2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

