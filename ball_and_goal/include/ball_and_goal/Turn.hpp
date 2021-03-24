#ifndef  GIRAR_BICA__TURN_HPP__
#define  GIRAR_BICA__TURN_HPP__

#include "bica/Component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace girar_bica
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

#endif //GIRAR_BICA__TURN_HPP__
