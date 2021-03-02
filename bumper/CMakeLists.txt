cmake_minimum_required(VERSION 3.0.2)
project(bumper)

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

add_executable(bumper_robot src/bumper_robot.cpp)
add_executable(bumper_new_version src/bumper_new_version.cpp)

target_link_libraries(bumper_robot ${catkin_LIBRARIES})
target_link_libraries(bumper_new_version ${catkin_LIBRARIES})

roslint_cpp(
 src/bumper_robot.cpp
 src/bumper_new_version.cpp
)
