#ifndef BALL_AND_GOAL__FINDBLUEGOAL_H__
#define BALL_AND_GOAL__FINDBLUEGOAL_H__

#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

#include <string>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "geometry_tf/transforms.h"

namespace bump_and_go_bica
{

class FindBlueGoal : public bica::Component
{
public:
    FindBlueGoal();
    void imageCb(const sensor_msgs::Image::ConstPtr& msg);
    void publish_detection(float x, float y);

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber image_sub_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster;

};

} // ball_and_goal

#endif // BALL_AND_GOAL__FINDBLUEGOAL_H__