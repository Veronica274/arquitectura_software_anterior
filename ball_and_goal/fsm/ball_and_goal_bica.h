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

*   THIS SOFTWARE IS PROVball_and_goal_bicaED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCball_and_goal_bicaENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
#ifndef BALL_AND_GOAL_BICA_H_
#define BALL_AND_GOAL_BICA_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class ball_and_goal_bica : public bica::Component
{
public:
  ball_and_goal_bica();
  virtual ~ball_and_goal_bica();

  void activateCode();

  	virtual void Go_yellow_code_iterative() {};
	virtual void Go_yellow_code_once() {};
	virtual void Turn_code_iterative() {};
	virtual void Turn_code_once() {};
	virtual void Go_blue_code_iterative() {};
	virtual void Go_blue_code_once() {};
	virtual void Go_ball_code_iterative() {};
	virtual void Go_ball_code_once() {};

  	virtual bool Turn_2_Go_yellow() {return false;};
	virtual bool Go_ball_2_Turn() {return false;};
	virtual bool Go_blue_2_Go_ball() {return false;};
	virtual bool Go_yellow_2_Go_blue() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void Go_yellow_activateDeps();
	void Turn_activateDeps();
	void Go_blue_activateDeps();
	void Go_ball_activateDeps();


  	static const int GO_YELLOW = 0;
	static const int TURN = 1;
	static const int GO_BLUE = 2;
	static const int GO_BALL = 3;

  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* BALL_AND_GOAL_BICA_H_ */
