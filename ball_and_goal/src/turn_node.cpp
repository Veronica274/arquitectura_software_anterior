#include "ros/ros.h"

#include "ball_and_goal/Turn.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turn");
  ros::NodeHandle n;

  ball_and_goal_bica::Turn turn;

  ros::Rate loop_rate(20);

  while ( turn.ok() )
  {
    turn.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
