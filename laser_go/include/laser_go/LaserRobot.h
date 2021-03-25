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

#ifndef LASER_GO_LASERROBOT_H
#define LASER_GO_LASERROBOT_H

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <random>
#include "cmath"

namespace laser_go
{
class LaserRobot
{
public:
    LaserRobot();
    virtual void step();
    virtual void markers();

protected:
    virtual void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    ros::NodeHandle n_;

    static const int GOING_FORWARD = 0;
    static const int BACK_TURNING_LEFT = 1;
    static const int BACK_TURNING_RIGHT = 2;

    int state_;

    double grados_centro_;
    bool centro_, derecha_, izquierda_;
    bool centro_laser_, derecha_laser_, izquierda_laser_;
    bool pressed_;

    ros::Time press_ts_;
    ros::Time turn_ts_;

    ros::Subscriber sub_laser_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_marker_array_;

    double turning_time_;
    double backing_time_;
};

}  // namespace laser_go

#endif  // LASER_GO_LASERROBOT_H
