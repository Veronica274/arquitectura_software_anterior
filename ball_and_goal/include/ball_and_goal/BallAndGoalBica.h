#ifndef BALL_AND_GOAL__BALL_AND_GOAL_BICA_H__
#define BALL_AND_GOAL__BALL_AND_GOAL_BICA_H__

#include "ball_and_goal_bica.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace ball_and_goal_bica 
{
class BallAndGoalBica : public bica::ball_and_goal_bica
{
public:
    BallAndGoalBica();

    bool Turn_2_Go_yellow();
	bool Go_ball_2_Turn();
	bool Go_blue_2_Go_ball();
	bool Go_yellow_2_Go_blue();


private:

    void CAMBIAR_callback(const std_msgs::Bool::ConstPtr msg);

    ros::NodeHandle nh_;
    ros::Subscriber obstacle_sub_;
    bool is_obstacle_;
};

} // ball_and_goal

#endif // BALL_AND_GOAL__BALL_AND_GOAL_BICA_H__