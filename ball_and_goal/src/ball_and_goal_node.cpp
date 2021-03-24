#include "ros/ros.h"
#include "ball_and_goal/BallAndGoalBica.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ball_and_goal");
  ros::NodeHandle n;

  ball_and_goal::BallAndGoalBica ball_and_goal;

  ros::Rate loop_rate(5);

  int count = 0;

  while (ball_and_goal.ok())
  {
      // Creo que hay que tener una funcion que se llama step()
      // me lo mirare por si acaso
    ball_and_goal.step()
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
