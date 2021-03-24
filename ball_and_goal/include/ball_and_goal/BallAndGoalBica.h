#ifndef BALL_AND_GOAL__BALL_AND_GOAL_BICA_H__
#define BALL_AND_GOAL__BALL_AND_GOAL_BICA_H__

#include "ball_and_goal.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
namespace ball_and_goal
{

class BallAndGoalBica : public bica::ball_and_goal
{
public:
    BallAndGoalBica();
    void obstacle_callback(const std_msgs::Bool::ConstPtr& msg);

    bool GoBlue_2_GoBall();
	bool GoBall_2_Turn(); 
	bool Turn_2_GoYellow(); 
	bool GoYellow_2_GoBlue(); 

private:
    ros::NodeHandle nh_;
    ros::Subscriber obstacle_sub_;

};

} // ball_and_goal

#endif // BALL_AND_GOAL__BALL_AND_GOAL_BICA_H__