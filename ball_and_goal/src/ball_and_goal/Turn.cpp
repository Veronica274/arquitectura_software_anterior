#include "ball_and_goal/Turn.h"

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace ball_and_goal_bica
{
Turn::Turn()
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
Turn::step()
{
    
    if(!isActive())
    {
        return;
    }
    
    geometry_msgs::Twist msg;


    msg.angular.z = 0.5;
    vel_pub_.publish(msg);
}

} //  girar_bica
