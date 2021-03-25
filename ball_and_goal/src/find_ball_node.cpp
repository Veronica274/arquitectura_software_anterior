#include "ros/ros.h"
#include "ball_and_goal/FindBall.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_blue_node");
  ros::NodeHandle n;

  ball_and_goal_bica::FindBall go_ball_node;

  ros::Rate loop_rate(5);

  int count = 0;

  while (go_ball_node.ok())
  {
    // Creo que si o si tiene haber una funcion step() en los .cpp
    // De todas formas me lo miraré manana en sus clases. 
    go_ball_node.imageCb();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
  
}