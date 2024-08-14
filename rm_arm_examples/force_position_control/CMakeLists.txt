cmake_minimum_required(VERSION 3.0.2)
project(force_position_control)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  rm_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include 
 ${catkin_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/include
)

add_executable(api_Force_Position_Control_demo src/api_force_position_control_demo.cpp)

target_link_libraries(api_Force_Position_Control_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
