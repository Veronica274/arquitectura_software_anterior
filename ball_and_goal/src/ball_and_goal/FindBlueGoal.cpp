#include "ball_and_goal/FindBlueGoal.h"


#include "geometry_msgs/Twist.h"
#include "bica/Component.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
namespace ball_and_goal
{

FindBlueGoal::FindBlueGoal(): it_(nh_)
{
    image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &FindBlueGoal::imageCb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
FindBlueGoal::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
    std_msgs::Bool msg;
    geometry_msgs::Twist msg2;
    int pos_x,pos_y;

    cv_bridge::CvImagePtr cv_ptr, cv_imageout;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//imagen que me acaba de llegar

    cv::Mat hsv;
    cv:cvtColor(cv_ptr->image , hsv, CV_RGB2HSV);

    int height = hsv.rows;
    int width = hsv.cols;
    int step = hsv.step;
    int channels = 3;  // RGB

    int x = 0;
    int y = 0;
    int counter = 0;
    for (int i=0; i < height; i++ ){
        for (int j=0; j < width; j++ )
        {
            int posdata = i * step + j * channels;
            
            if((hsv.data[posdata] >= 0) && (hsv.data[posdata] <= 153) && (hsv.data[posdata+1]  >= 176) && (hsv.data[posdata+1] <= 223) && (hsv.data[posdata+2]  >=70) && (hsv.data[posdata+2] <= 92))
            {
                x += j;
                y += i;
                counter++;
            } 
        }
    }
    if (counter > 0){
        //ROS_INFO("Ball at %d %d", x / counter , y / counter);
        pos_x = x / counter;
        pos_y = y / counter;
       
        msg2.angular.z = 0.5;
        if(pos_x >= 200 && pos_x <= 300)
        {
            if (pos_y <= 100)
            {
                msg2.linear.x = 0.0;
            }
            else 
            {
                msg2.linear.x = 0.2;
            }
            msg2.angular.z = 0.0;
            is_obstacle_ = true;         
        }
        
    } else {
        ROS_INFO("NO BALL FOUND");
    }

    vel_pub_.publish(msg2);

}

void
FindBlueGoal::step()
{
    if(!isActive()){
        return;
    }
    geometry_msgs::Twist msg;

    msg.linear.x = -0.5;

    vel_pub_.publish(msg);
}


} // ball_and_goal

