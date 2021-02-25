cmake_minimum_required(VERSION 3.0.2)
project(laser_go)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_robot src/laser_robot.cpp)

target_link_libraries(laser_robot ${catkin_LIBRARIES})

roslint_cpp(
 src/laser_robot.cpp
)
