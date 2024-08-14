cmake_minimum_required(VERSION 3.0.2)
project(control_arm_move)

# find_package(OpenCV REQUIRED)

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

add_executable(api_Control_Arm_Move_demo src/api_move_demo.cpp)

target_link_libraries(api_Control_Arm_Move_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

