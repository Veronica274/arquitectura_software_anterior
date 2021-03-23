cmake_minimum_required(VERSION 3.0.2)
project(ball_and_goal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  bica
  image_transport
  cv_bridge
  std_msgs
  roslint
  kobuki_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  bica
  std_msgs
  kobuki_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV)
include_directories(
  include
  fsm
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/FindBall.cpp
   fsm/ball_and_goal.cpp
 )


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(find_ball_node src/find_ball_node.cpp)
add_dependencies(find_ball_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_ball_node ${PROJECT_NAME} ${catkin_LIBRARIES})


roslint_cpp(
 src/find_ball_node.cpp
)








