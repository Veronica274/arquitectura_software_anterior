// Copyright 2021 Ruben Montilla, Víctor de la Torre, MArio Esteban y Veronica Tornero
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
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

#define BACKWARD_TIME 2.0

class BumperRobot
{
public:
  BumperRobot()
  {
    bumper_pressed_ = false;
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    bumper_sub_ = n_.subscribe("/mobile_base/events/bumper", 1, &BumperRobot::messageCallback, this);
  }
  void messageCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    bumper_pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;
    if (bumper_pressed_) { bumper_pressed_ = true; }

    else if (!bumper_pressed_) { bumper_pressed_ = false; }
  }

  void actions()
  {
    geometry_msgs::Twist vel;

    if (bumper_pressed_)
    {
      vel.linear.x = 0.0;
    }
    else if (!bumper_pressed_)
    {
      vel.linear.x = 0.3;
    }

    vel_pub_.publish(vel);
  }

private:
  ros::NodeHandle n_;

  bool bumper_pressed_;

  ros::Subscriber bumper_sub_;
  ros::Publisher vel_pub_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bumper_robot");
  BumperRobot bumperrobot;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    bumperrobot.actions();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
