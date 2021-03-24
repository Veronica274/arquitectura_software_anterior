#ifndef BALL_AND_GOAL__FINDBALL_H__
#define BALL_AND_GOAL__FINDBALL_H__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

namespace bump_and_go_bica
{

class FindBall : public bica::Component
{
public:
    FindBall();
    void imageCb(const sensor_msgs::Image::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber image_sub_;
    ros::Publisher obstacle_pub_;

    bool is_obstacle_;

};

} // ball_and_goal

#endif // BALL_AND_GOAL__FINDBALL_H__