#include "ros/ros.h"

#include "girar_bica/Turn.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turn");
  ros::NodeHandle n;

  girar_bica::Turn turn;

  ros::Rate loop_rate(20);

  while ( turn.ok() )
  {
    turn.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
