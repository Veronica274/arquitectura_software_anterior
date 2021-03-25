#include "ros/ros.h"
#include "ball_and_goal/FindBlueGoal.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_blue_node");
  ros::NodeHandle n;

  ball_and_goal_bica::FindBlueGoal go_blue_node;

  ros::Rate loop_rate(5);

  int count = 0;

  while (go_blue_node.ok())
  {
    // Creo que si o si tiene haber una funcion step() en los .cpp
    // De todas formas me lo miraré manana en sus clases. 
    go_blue_node.step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
  
}