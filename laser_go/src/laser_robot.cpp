// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"


#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h" 
#include "visualization_msgs/Marker.h" 
#include "visualization_msgs/MarkerArray.h" 

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0
#define MIN_DIST 0.3;

class LaserGo
{
public:
  LaserGo(): state_(GOING_FORWARD), pressed_(false)
  {
    sub_laser_ = n_.subscribe("/scan", 1, &LaserGo::laserCallback, this);
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pub_marker_ = n_.advertise<visualization_msgs::Marker>("/visualization_markers", 1);
    pub_marker_array_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_markers_array", 1);
  }

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    //En esta variable almacenaremos los grados del centro para que nos sea mas fácil operar con ángulos
    grados_centro = msg->ranges.size()/2 * msg->angle_increment * (-1);
    centro_ = msg->ranges[msg->ranges.size()/2] < MIN_DIST;
    izquierda_ = msg->ranges[(grados_centro - (M_PI/5))/((-1)*msg->angle_increment)] < MIN_DIST;
    derecha_ = msg->ranges[(grados_centro + (M_PI/5))/((-1)*msg->angle_increment)] < MIN_DIST;
    pressed_ = (centro_ || derecha_ || izquierda_);
    
    ROS_INFO("Data centro: [%i][%f][%ld]",centro_, msg->ranges[msg->ranges.size()/2], msg->ranges.size()/2);
    ROS_INFO("Data derecha: [%i][%f][%f]",derecha_, msg->ranges[(grados_centro + (M_PI/5))/((-1)*msg->angle_increment)], (grados_centro + (M_PI/5))/((-1)*msg->angle_increment));
    ROS_INFO("Data izquierda: [%i][%f][%f]",izquierda_, msg->ranges[(grados_centro - (M_PI/5))/((-1)*msg->angle_increment)], (grados_centro - (M_PI/5))/((-1)*msg->angle_increment));
  }

    //Las variables y estructura se modificarán para que funcione con el laser y no con el bumper
  void step()
  {
    geometry_msgs::Twist cmd;

    switch (state_)
    {
    case GOING_FORWARD:
      cmd.linear.x=0.3;
      cmd.angular.z=0.0;

      if (pressed_)
      {
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }
      break;

    case GOING_BACK:

      cmd.linear.x=-0.3;
      cmd.angular.z=0.0;
      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        state_ = TURNING;
        ROS_INFO("GOING_BACK -> TURNING");
      }
      break;

    case TURNING:
      cmd.linear.x=0.0;
      cmd.angular.z=0.3;
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }
    //pub_vel_.publish(cmd);
  }
  void markers()
  {
    visualization_msgs::Marker marker_centro;

    marker_centro.header.frame_id = "base_link";
    marker_centro.header.stamp = ros::Time();
    marker_centro.ns = "my_namespace";
    marker_centro.id = 0;
    marker_centro.type = visualization_msgs::Marker::SPHERE;
    marker_centro.action = visualization_msgs::Marker::ADD;
    marker_centro.pose.position.x = 1;
    marker_centro.pose.position.y = 0;
    marker_centro.pose.position.z = 0;
    marker_centro.pose.orientation.x = 0.0;
    marker_centro.pose.orientation.y = 0.0;
    marker_centro.pose.orientation.z = 0.0;
    marker_centro.pose.orientation.w = 1.0;
    marker_centro.scale.x = 0.25;
    marker_centro.scale.y = 0.25;
    marker_centro.scale.z = 0.25;
    marker_centro.color.a = 1.0; 
    marker_centro.color.r = 0.0;
    marker_centro.color.g = 1.0;
    marker_centro.color.b = 0.0;
    marker_centro.lifetime = ros::Duration(1.0);

    visualization_msgs::MarkerArray msg_array;
    msg_array.markers.resize(3);
    msg_array.markers[0] = marker_centro;

    visualization_msgs::Marker marker_izq;

    marker_izq = marker_centro;
    marker_izq.header.frame_id = "base_link";
    marker_izq.id = 1;
    marker_izq.pose.position.x = cos(M_PI/5);
    marker_izq.pose.position.y = sin(M_PI/5);
    msg_array.markers[1] = marker_izq;

    visualization_msgs::Marker marker_derecha;
    marker_derecha = marker_centro;
    marker_derecha.header.frame_id = "base_link";
    marker_derecha.id = 2;
    marker_derecha.pose.position.x = (cos(M_PI/5));
    marker_derecha.pose.position.y = sin(M_PI/5)*(-1);
   
    msg_array.markers[2] = marker_derecha;

    if(centro_)
    {
      marker_centro.color.g = 0.0;
      marker_centro.color.r = 1.0;
    }
    else if(derecha_)
    {
      marker_derecha.color.g = 0.0;
      marker_derecha.color.r = 1.0;
    }
    else if(izquierda_)
    {
      marker_izq.color.g = 0.0;
      marker_izq.color.r = 1.0;
    }

    pub_marker_array_.publish(msg_array);
  }

private:
  ros::NodeHandle n_;

  static const int GOING_FORWARD = 0;
  static const int GOING_BACK = 1;
  static const int TURNING = 2;

  int state_;

  double grados_centro;
  bool centro_, derecha_, izquierda_;
  bool pressed_;

  ros::Time press_ts_;
  ros::Time turn_ts_;

  ros::Subscriber sub_laser_;
  ros::Publisher pub_vel_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_marker_array_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lasergo");

  LaserGo lasergo;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    lasergo.step();
    lasergo.markers();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
