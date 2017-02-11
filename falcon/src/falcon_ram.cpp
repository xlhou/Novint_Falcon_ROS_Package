/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32MultiArray.h"
// %EndTag(MSG_HEADER)%
#include "FalconSDK.h"
#include "FalconPID.h"

#include <unistd.h>
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <vector>
#include<sys/stat.h>
#include<sys/types.h>

#define VEL2FALCON_SCALE 0.4//0.3		//base on 1m/s max speed
#define MAX_DISP 0.4			//0.053

using namespace std;
using namespace libnifalcon;

double pos[3]={0,0,0},ppos[3]={0,0,0};
double pos_buffer[3]={0,0,0};
double rotation=0;
double envforce[3]={0,0,0};

double out_force[4]={0,0,0,0};

std_msgs::Float64MultiArray force_msg;

void VelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	pos_buffer[0] = msg->data[7]*VEL2FALCON_SCALE;		//Y in falcon frame
	pos_buffer[1] = msg->data[8]*VEL2FALCON_SCALE;		//x in falcon frame
	pos_buffer[2] = -msg->data[6]*VEL2FALCON_SCALE;		//z in falcon frame
}



int main(int argc, char **argv)
{
	FalconPID fpid;		// class declarition
	fpid.option=2;		// enable the velocity mapping under admittance mode

	fpid.fal_open();	// open device 
	fpid.init();		// initiating device

// %Tag(INIT)%
	ros::init(argc, argv, "falcon");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
	ros::NodeHandle n;
// %EndTag(NODEHANDLE)%
 
// %Tag(PUBLISHER)%
	ros::Publisher falcon_pub = n.advertise<std_msgs::Float64MultiArray>("VICON_CON", 100);
	ros::Subscriber vel_sub = n.subscribe("vehicle_state", 100, VelCallback);
// %EndTag(PUBLISHER)%
	

// %Tag(LOOP_RATE)%
	ros::Rate loop_rate(2000);
	double time = ros::Time::now().toSec();
// %EndTag(LOOP_RATE)%

// %Tag(ROS_OK)%
	int count = 0;
	while (ros::ok())
	{
// %EndTag(ROS_OK)%
		static double buf_eforce[3]={0,0,0};
		if (pos_buffer[0]>MAX_DISP)
		{	
			pos[0] = MAX_DISP;
		}
		else
		{
			if (pos_buffer[0]<-MAX_DISP)
			{
				pos[0] = -MAX_DISP;
			}
			else
			{
				pos[0] = pos_buffer[0];
			}
		}
		if( pos_buffer[1]>MAX_DISP)
		{	
			pos[1] = MAX_DISP;
		}
		else
		{
			if( pos_buffer[1]<-MAX_DISP)
			{
				pos[1] = -MAX_DISP;
			}
			else
			{
				pos[1] = pos_buffer[1];
			}
		}

		if( pos_buffer[2]>MAX_DISP)
		{	
			pos[2] = MAX_DISP;//0.057;
		}
		else
		{
			if( pos_buffer[2]<-MAX_DISP)
			{
				pos[2] = -MAX_DISP;
			}
			else
			{
				pos[2] = pos_buffer[2];
			}
		};
		
		pos[2] = pos_buffer[2];
		
		fpid.run(pos, out_force, buf_eforce, ppos); // call falcon pid controller, servo the velocity position, and measure the force input from user

		if (ppos[3]<0){
			ppos[3] += 2*PI;
		}else{
			if (ppos[3]>2*PI){
				ppos[3] -= 2*PI; 
			}
		}
		cout<<"px: "<<(0.125-ppos[2])*20<<",py: "<<ppos[0]*20<<",pz: "<<ppos[1]*12<<"yaw: "<<ppos[3]<<endl;

/*
		force_msg.data.push_back((0.125-ppos[2])*40);// x in body fixed frame
		force_msg.data.push_back(ppos[0]*40);// y in body fixed frame
		force_msg.data.push_back(ppos[1]*24);// z in body fixed frame
		force_msg.data.push_back(ppos[3]);// yaw in body fixed frame
		force_msg.data.push_back(time);
*/

		time = ros::Time::now().toSec();

		force_msg.data.push_back(-out_force[2]/4.0*2.0);//x in body fixed frame,maximum force 4.0N, maximum velocity 1.0m/s
		force_msg.data.push_back(out_force[0]/4.0*2.0);// y in body fixed frame
		force_msg.data.push_back(0);// 0
		force_msg.data.push_back(ppos[3]);// yaw in body fixed frame
		force_msg.data.push_back(out_force[1]/4.0*2.0);// z in body fixed frame
		force_msg.data.push_back(time);


// %Tag(PUBLISH)%
		falcon_pub.publish(force_msg);
// %EndTag(PUBLISH)%
		force_msg.data.clear();
// %Tag(SPINONCE)%
		ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
		loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
		++count;
	}

	fpid.fal_close();
	return 0;
}
// %EndTag(FULLTEXT)%
