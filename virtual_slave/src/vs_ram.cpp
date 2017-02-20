#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"


#define kp 2
#define ki 0
#define kd 0

float vel_ref[4] = {0};// vel reference
float vel[4] = {0};// vel of the virtual slave
float m_vs = 1.2;// mass of the virtual slave
float err[4] = {0}, i_err[4] = {0}, d_err[4] = {0}, p_err[4] = {0}, u[4] = {0};//controller variables


void VelCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	vel_ref[0] = msg->data[0];
	vel_ref[1] = msg->data[1];
	vel_ref[2] = msg->data[4];
	vel_ref[3] = msg->data[3];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vs_ram");

	ros::NodeHandle n;

	ros::Publisher vs_pub = n.advertise<std_msgs::Float32MultiArray>("vehicle_state", 10);
	ros::Subscriber vs_sub = n.subscribe("VICON_CON", 10, VelCallback);

	ros::Rate loop_rate(1000);

	int count = 0;
	while (ros::ok())
	{
		for(int i = 0; i<4; i++){
			err[i] = vel_ref[i] - vel[i];
			d_err[i] = -(err[i] - p_err[i])/0.001;
			i_err[i] += err[i];
			u[i] = kp * err[i] + ki * i_err[i] + kd * d_err[i];
			p_err[i] = err[i];
			vel[i] += u[i]/m_vs * 0.001;
		}

		std_msgs::Float32MultiArray vel_msg;

		vel_msg.data.push_back(0);
		vel_msg.data.push_back(0);
		vel_msg.data.push_back(0);
		vel_msg.data.push_back(0);
		vel_msg.data.push_back(0);
		vel_msg.data.push_back(0);
		vel_msg.data.push_back(vel[0]);
		vel_msg.data.push_back(vel[1]);
		vel_msg.data.push_back(vel[2]);

		ROS_INFO("\nvxr: %f,vyr: %f,vzr: %f,yawr: %f;\n vx: %f, vy: %f, vz: %f, yaw: %f", vel_ref[0], vel_ref[1], vel_ref[2], vel_ref[3], vel[0], vel[1], vel[2], vel[3]);

		vs_pub.publish(vel_msg);
	
		ros::spinOnce();
	
		loop_rate.sleep();
		++count;
	}

	return 0;
}
