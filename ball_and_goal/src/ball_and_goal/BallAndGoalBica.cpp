# include "ros/ros.h"
# include "ball_and_goal/BallAndGoal.h"

# define MAX_TIME

namespace ball_and_goal
{
BallAndGoalBica::BallAndGoalBica()
{
}

bool
BallAndGoal::Go_yellow_2_Go_blue()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoal::Go_blue_2_Go_ball()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}

bool
BallAndGoal::Go_ball_2_Turn()
{
    return (ros::Time::now() - state_ts_).toSec() > 5.0;
}

bool
BallAndGoal::Turn_2_Go_yellow()
{
    return (ros::Time::now() - state_ts_).toSec() > MAX_TIME;
}
}