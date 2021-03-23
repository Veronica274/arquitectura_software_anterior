cmake_minimum_required(VERSION 3.0.2)
project(ball_and_goal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs image_transport cv_bridge
)

find_package(OpenCV)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(find_yellow_goal src/find_yellow_goal.cpp)

target_link_libraries(find_yellow_goal ${catkin_LIBRARIES})

roslint_cpp(
 src/find_yellow_goal.cpp
)
