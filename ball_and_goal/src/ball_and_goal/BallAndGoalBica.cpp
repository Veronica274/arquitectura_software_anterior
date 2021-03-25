# include "ros/ros.h"
# include "ball_and_goal/BallAndGoalBica.h"

# define MAX_TIME 30.0

namespace ball_and_goal_bica 
{

BallAndGoalBica::BallAndGoalBica() 
{
    obstacle_sub_ = nh_.subscribe("obstacle", 10, &BallAndGoalBica::CAMBIAR_callback, this);
}


//Callback de prueba, hay que cambiar
void
BallAndGoalBica::CAMBIAR_callback(const std_msgs::Bool::ConstPtr msg)
{
    is_obstacle_ = msg->data;

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