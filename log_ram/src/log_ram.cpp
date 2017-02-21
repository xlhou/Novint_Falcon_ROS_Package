#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/TransformStamped.h"

#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>


#include <sys/types.h>
#include <sys/stat.h>

using namespace std;

FILE* fid_falcon;
char dir[100]="";
double vel[4] = {0}, vel_ref[7] ={0};

void log_falcon()
{
	char log[100]="";
	char format[30]="/falcon.txt";
	strcat(log,dir);
	strcat(log,format);
	
	if((fid_falcon=fopen(log,"w+"))==NULL)
		 cout<<"Fail to open the falcon log file!"<<endl;
}

void logging()
{
	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

//you need to change the path to the log directory 
	if (timeinfo->tm_sec<55)
	{
	sprintf ( dir ,"/home/pilot/catkin_ws/log/%4d_%02d_%02d_%02d%02d",  timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
	}
	else
	{
		sprintf ( dir ,"/home/pilot/catkin_ws/log/%4d_%02d_%02d_%02d%02d",  timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min+1);

	}

	mkdir(dir,0777);//creating a directory
	log_falcon();
}

void VelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	vel[0] = msg->data[6];//vx
	vel[1] = msg->data[7];//vy
	vel[2] = msg->data[8];//vz
	vel[3] = msg->data[5];//yaw
}

void RefCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	vel_ref[0] = msg->data[0];//vxr
	vel_ref[1] = msg->data[1];//vyr
	vel_ref[2] = msg->data[4];//vzr
	vel_ref[3] = msg->data[3];//yawr
	vel_ref[4] = msg->data[5];//px
	vel_ref[5] = msg->data[6];//py
	vel_ref[6] = msg->data[7];//pz

	ROS_INFO("\nvxr: %1.3f,vyr: %1.3f,vzr: %1.3f,yawr: %1.3f\n px: %1.3f, py: %1.3f, pz: %1.3f",vel_ref[0],vel_ref[1],vel_ref[2],vel_ref[3],vel_ref[4],vel_ref[5],vel_ref[6]);

	fseek(fid_falcon,0,SEEK_END);
	fprintf(fid_falcon,"%f",ros::Time::now().toSec());
	fseek(fid_falcon,0,SEEK_END);
	fprintf(fid_falcon,",%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f;\n",vel_ref[0],vel_ref[1],vel_ref[2],vel_ref[3],vel_ref[4],vel_ref[5],vel_ref[6],vel[0],vel[1],vel[2],vel[3]);	
	fseek(fid_falcon,0,SEEK_END);
}

int main(int argc, char **argv)
{
	logging();

	ros::init(argc, argv, "ram_logging");

	ros::NodeHandle log;

	ros::Subscriber vel_sub = log.subscribe("vehicle_state", 10, VelCallback);
	ros::Subscriber ref_sub = log.subscribe("VICON_CON", 10, RefCallback);
	ros::spin();

	return 0;
}

