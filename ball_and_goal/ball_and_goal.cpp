/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVball_and_goalED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCball_and_goalENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "ball_and_goal.h"

namespace bica
{
ball_and_goal::ball_and_goal() : state_(GOYELLOW), myBaseId_("ball_and_goal")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

ball_and_goal::~ball_and_goal()
{
}

void ball_and_goal::activateCode()
{
  	deactivateAllDeps();

	state_ = GOYELLOW;
	state_ts_ = ros::Time::now();

	GoYellow_activateDeps();
	GoYellow_code_once();

}

bool ball_and_goal::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case GOBALL:

	GoBall_code_iterative();

	msg.data = "GoBall";
	if(GoBall_2_Turn())
	{

	deactivateAllDeps();

	state_ = TURN;
	state_ts_ = ros::Time::now();

	Turn_activateDeps();
	Turn_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GOYELLOW:

	GoYellow_code_iterative();

	msg.data = "GoYellow";
	if(GoYellow_2_GoBlue())
	{

	deactivateAllDeps();

	state_ = GOBLUE;
	state_ts_ = ros::Time::now();

	GoBlue_activateDeps();
	GoBlue_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TURN:

	Turn_code_iterative();

	msg.data = "Turn";
	if(Turn_2_GoYellow())
	{

	deactivateAllDeps();

	state_ = GOYELLOW;
	state_ts_ = ros::Time::now();

	GoYellow_activateDeps();
	GoYellow_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GOBLUE:

	GoBlue_code_iterative();

	msg.data = "GoBlue";
	if(GoBlue_2_GoBall())
	{

	deactivateAllDeps();

	state_ = GOBALL;
	state_ts_ = ros::Time::now();

	GoBall_activateDeps();
	GoBall_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
ball_and_goal::deactivateAllDeps()
{
	removeDependency("go_yellow");
	removeDependency("find_yellow");
	removeDependency("go_ball");
	removeDependency("go_blue");
	removeDependency("turn");
	removeDependency("find_blue");
	removeDependency("find_ball");
};

void
ball_and_goal::GoBall_activateDeps()
{
	addDependency("find_ball");
	addDependency("go_ball");
}

void
ball_and_goal::GoYellow_activateDeps()
{
	addDependency("go_yellow");
	addDependency("find_yellow");
}

void
ball_and_goal::Turn_activateDeps()
{
	addDependency("turn");
}

void
ball_and_goal::GoBlue_activateDeps()
{
	addDependency("go_blue");
	addDependency("find_blue");
}



} /* namespace bica */
