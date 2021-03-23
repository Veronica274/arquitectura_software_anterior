#ifndef BALL_AND_GOAL_FIND_OBJECT_H_
#define BALL_AND_GOAL_FIND_OBJECT_H_

#include "Bica/component.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace ball_and_goal
{
class FindObject : public bica::component
{
public:
    FindObject(): it_(nh_), is_object_(false)
    {
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &FindObject::imageCb, this);
        image_pub_ = it_.advertise("/hsv/image_filtered", 1);
        hsv_sub_ = it_.subscribe("/hsv/image_filtered", 1, &FindObject::hsvCb, this);
    }
    
    virtual void FindObject::imageCb(){};
    virtual void FindObject::hsvCb(){};

private:
    ros::NodeHandle nh_;

    ros::Subscriber image_sub_;
    ros::Publisher image_sub_;
    ros::Subscriber hsv_sub_;

    bool is_object_;
}
}

#endif // BALL_AND_GOAL_FIND_OBJECT_H_
