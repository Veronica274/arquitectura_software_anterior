// Copyright 2021 ROScon de Reyes
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

#include <random>
#include "cmath"

#include "laser_go/LaserRobot.h"

#define MIN_DIST 0.4
#define VARIANCE 0.2
#define MEAN 1

namespace laser_go
{
LaserRobot::LaserRobot()
{
    state_ = GOING_FORWARD;
    sub_laser_ = n_.subscribe("/scan", 1, &LaserRobot::laserCallback, this);
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    pub_marker_array_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_markers_array", 1);
}

void LaserRobot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // En esta variable almacenaremos los grados del centro para que nos sea mas fácil operar con ángulos
    // Como el incremento del ángulo es negativo, lo haremos positivo para que las operaciones nos den bien.
    grados_centro_ = msg->ranges.size()/2 * msg->angle_increment * (-1);
    centro_ = msg->ranges[msg->ranges.size()/2] < MIN_DIST;
    izquierda_ = msg->ranges[(grados_centro_ - (M_PI/5))/(-msg->angle_increment)] < MIN_DIST;
    derecha_ = msg->ranges[(grados_centro_ + (M_PI/5))/(-msg->angle_increment)] < MIN_DIST;
}

void LaserRobot::step()
{
    geometry_msgs::Twist cmd;

    srand(time(NULL));

    std::default_random_engine generator(time(0));
    std::normal_distribution<double> distribution(MEAN, VARIANCE);

    backing_time_ = distribution(generator);
    // Se suma backing_time para que gira el tiempo determinado
    turning_time_ = distribution(generator) + backing_time_;

    switch (state_)
    {
      case GOING_FORWARD:
        cmd.linear.x = 0.3;
        cmd.angular.z = 0.0;
        if (izquierda_)
        {
          cmd.linear.x = 0.0;
          izquierda_laser_ = 1;
          press_ts_ = ros::Time::now();
          state_ = BACK_TURNING_LEFT;
        }
        else if (derecha_)
        {
          cmd.linear.x = 0.0;
          derecha_laser_ = 1;
          press_ts_ = ros::Time::now();
          state_ = BACK_TURNING_RIGHT;
        }
        else if (centro_)
        {
          cmd.linear.x = 0.0;
          centro_laser_ = 1;
          press_ts_ = ros::Time::now();
          int random = rand() % 2;
          if (random == 0)
          {
            state_ = BACK_TURNING_RIGHT;
          }
          else if (random == 1)
          {
            state_ = BACK_TURNING_LEFT;
          }
        }
        break;

      case BACK_TURNING_RIGHT:
        if ((ros::Time::now() - press_ts_).toSec() < backing_time_ )
        {
          cmd.linear.x = -0.3;
          cmd.angular.z = 0.0;
        }
        else if ((ros::Time::now() - press_ts_).toSec() < turning_time_ )
        {
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.3;
        }
        else
        {
          centro_laser_ = 0;
          derecha_laser_ = 0;
          state_ = GOING_FORWARD;
        }
        break;

      case BACK_TURNING_LEFT:
        if ((ros::Time::now() - press_ts_).toSec() < backing_time_ )
        {
          cmd.linear.x = -0.3;
          cmd.angular.z = 0.0;
        }
        else if ((ros::Time::now() - press_ts_).toSec() < turning_time_ )
        {
          cmd.linear.x = 0.0;
          cmd.angular.z = -0.3;
        }
        else
        {
          centro_laser_ = 0;
          izquierda_laser_ = 0;
          state_ = GOING_FORWARD;
        }
        break;
    }
    pub_vel_.publish(cmd);
}

void LaserRobot::markers()
{
    visualization_msgs::Marker marker_centro;

    marker_centro.header.frame_id = "base_link";
    marker_centro.header.stamp = ros::Time();
    marker_centro.ns = "my_namespace";
    marker_centro.id = 0;
    marker_centro.type = visualization_msgs::Marker::SPHERE;
    marker_centro.action = visualization_msgs::Marker::ADD;
    marker_centro.pose.position.x = MIN_DIST;
    marker_centro.pose.position.y = 0;
    marker_centro.pose.position.z = 0;
    marker_centro.pose.orientation.x = 0.0;
    marker_centro.pose.orientation.y = 0.0;
    marker_centro.pose.orientation.z = 0.0;
    marker_centro.pose.orientation.w = 1.0;
    marker_centro.scale.x = MIN_DIST / 2;
    marker_centro.scale.y = MIN_DIST / 2;
    marker_centro.scale.z = MIN_DIST / 2;
    marker_centro.color.a = 1.0;
    marker_centro.color.r = 0.0;
    marker_centro.color.g = 1.0;
    marker_centro.color.b = 0.0;
    marker_centro.lifetime = ros::Duration(1.0);

    visualization_msgs::MarkerArray msg_array;
    msg_array.markers.resize(3);

    visualization_msgs::Marker marker_izq;

    marker_izq = marker_centro;
    marker_izq.header.frame_id = "base_link";
    marker_izq.id = 1;
    marker_izq.pose.position.x = cos(M_PI / 5) * MIN_DIST;
    marker_izq.pose.position.y = sin(M_PI / 5) * MIN_DIST;

    visualization_msgs::Marker marker_derecha;
    marker_derecha = marker_centro;
    marker_derecha.header.frame_id = "base_link";
    marker_derecha.id = 2;
    marker_derecha.pose.position.x = cos(M_PI / 5) * MIN_DIST;
    marker_derecha.pose.position.y = sin(M_PI / 5) * -MIN_DIST;
    if (centro_laser_)
    {
      marker_centro.color.g = 0.0;
      marker_centro.color.r = 1.0;
    }
    else if (derecha_laser_)
    {
      marker_derecha.color.g = 0.0;
      marker_derecha.color.r = 1.0;
    }
    else if (izquierda_laser_)
    {
      marker_izq.color.g = 0.0;
      marker_izq.color.r = 1.0;
    }

    msg_array.markers[0] = marker_centro;
    msg_array.markers[1] = marker_izq;
    msg_array.markers[2] = marker_derecha;
    pub_marker_array_.publish(msg_array);
}

}  // namespace laser_go
