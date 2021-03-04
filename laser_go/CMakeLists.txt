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
  include
  ${catkin_INCLUDE_DIRS} 
)

add_library(laser_go src/laser_go/LaserRobot.cpp)

set(LINK_LIBS
  laser_go
  ${catkin_LIBRARIES}
)

add_executable(laser_robot src/laser_robot.cpp)
target_link_libraries(laser_robot ${catkin_LIBRARIES})

add_executable(laser_robot_executable src/laser_robot_executable.cpp src/laser_go/LaserRobot.cpp)
target_link_libraries(laser_robot_executable ${catkin_LIBRARIES})


roslint_cpp(
 src/laser_robot.cpp
 src/laser_robot_executable.cpp
)
