# include "ros/ros.h"
# include "ball_and_goal/BallAndGoalBica.h"

# define MAX_TIME 30.0

namespace ball_and_goal_bica 
{

BallAndGoalBica::BallAndGoalBica() 
{
}

bool
BallAndGoalBica::Go_yellow_2_Go_blue()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoalBica::Go_blue_2_Go_ball()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoalBica::Go_ball_2_Turn()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoalBica::Turn_2_Go_yellow()
{
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
}

}  // ball_and_goal