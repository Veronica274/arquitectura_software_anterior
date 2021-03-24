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
#include "laser_go/LaserRobot.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lasergo");

  laser_go::LaserRobot lg;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    lg.step();
    lg.markers();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
