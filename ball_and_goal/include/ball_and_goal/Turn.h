#ifndef  BALL_AND_GOAL__TURN_H__
#define  BALL_AND_GOAL__TURN_H__

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace ball_and_goal_bica
{

class Turn : public bica::Component
{
public:
    Turn();
    void step();

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;

};

}  //  girar_bica

#endif //BALL_AND_GOAL__TURN_HPP__
