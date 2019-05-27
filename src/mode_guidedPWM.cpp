#include "ros/ros.h"
#include <cstdlib>

#include <tf/transform_datatypes.h>
#include <mavros/mavros.h>
#include <mavros_msgs/CommandLong.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>


ros::ServiceServer serviceCommand;
ros::Publisher thruster_command_pub;
ros::Publisher joint_setpoint_pub;
ros::Subscriber state_sub;
ros::Publisher local_position_pub;
ros::Publisher compass_pub;


int idDir = 0;
float leftPWM  = 0;
float rightPWM = 0;
std::string frameId="world";
std::string airboatJointName="fwd_joint";
std::string airboatLinkName="fwd";
std::string diffboatLeftLinkName="fwd_left";
std::string diffboatRightLinkName="fwd_right";
std::string boatNamespace="/diffboat/";
//------------------------------AIRBOAT------------------------------
//VALUE FROM REAL BOAT (PWM)
float i_minPWM1     = 1220;
float i_nothingPWM1 = 1575;
float i_maxPWM1     = 1900;
//VALUE FOR SIMULATED BOAT (radians)
float o_minPWM1     = -0.5;
float o_nothingPWM1 =  0.0;
float o_maxPWM1     = +0.5;

//VALUE FROM REAL BOAT (PWM)
float i_minPWM3     =  900;
float i_nothingPWM3 = 1100;
float i_maxPWM3     = 1350;
//VALUE FOR SIMULATED BOAT (FORCE IN NEWTONS)
float o_minPWM3     = -3.13;
float o_nothingPWM3 =   0.0;
float o_maxPWM3     = +3.13;

//------------------------------DIFFBOAT------------------------------
//VALUE FROM REAL BOAT  (PWM)
float i_minPWM2     = 1400;
float i_nothingPWM2 = 1500;
float i_maxPWM2     = 1600;
//VALUE FOR SIMULATED BOAT  (FORCE IN NEWTONS)
float o_minPWM2     =-11.5;
float o_nothingPWM2 =  0.0;
float o_maxPWM2     =+11.5;

//VALUE FROM REAL BOAT  (PWM)
float i_minPWM4     = 1400;
float i_nothingPWM4 = 1500;
float i_maxPWM4     = 1600;
//VALUE FOR SIMULATED BOAT  (FORCE IN NEWTONS)
float o_minPWM4     = -11.5;
float o_nothingPWM4 =   0.0;
float o_maxPWM4     = +11.5;

geometry_msgs::PoseStamped msg_boatLocalPose;
std_msgs::Float64 compass_hdg;



bool receiveCommandLong(mavros_msgs::CommandLong::Request  &req,
		mavros_msgs::CommandLong::Response &res)
{
	if (req.command==183)
	{
		// AIRBOAT
		if (req.param1==1)
		{
			// direcao
			sensor_msgs::JointState msg;
			msg.header.seq = idDir;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id="world";
			msg.name.push_back(airboatJointName);
			float angle = req.param3-i_nothingPWM1;
			if (angle>=0)
				angle=(o_maxPWM1-o_nothingPWM1)*angle/(i_maxPWM1-i_nothingPWM1);
			else
				angle=(o_minPWM1-o_nothingPWM1)*angle/(i_minPWM1-i_nothingPWM1);
			msg.position.push_back(angle);
			joint_setpoint_pub.publish(msg);
			res.success= true;
			res.result=1;
			return true;
		}
		if (req.param1==3)
		{
			// propulsor
			sensor_msgs::JointState msg;
			msg.header.seq = idDir;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id="world";
			msg.name.push_back(airboatLinkName);
			float force = req.param3-i_nothingPWM3;
			if (force>=0)
				force=(o_maxPWM3-o_nothingPWM3)*force/(i_maxPWM3-i_nothingPWM3);
			else
				force=(o_minPWM3-o_nothingPWM3)*force/(i_minPWM3-i_nothingPWM3);
			msg.position.push_back(force);
			thruster_command_pub.publish(msg);
			res.success= true;
			res.result=1;
			return true;
		}
		// DIFFBOAT
		if (req.param1==2)
		{
			float force = req.param3-i_nothingPWM2;
			if (force>=0)
				force=(o_maxPWM2-o_nothingPWM2)*force/(i_maxPWM2-i_nothingPWM2);
			else
				force=(o_minPWM2-o_nothingPWM2)*force/(i_minPWM2-i_nothingPWM2);

			leftPWM=force;
			
			// propulsor
			sensor_msgs::JointState msg;
			msg.header.seq = idDir;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id="world";
			msg.name.push_back(diffboatLeftLinkName);
			msg.name.push_back(diffboatRightLinkName);
			msg.position.push_back(leftPWM);
			msg.position.push_back(rightPWM);
			thruster_command_pub.publish(msg);
			res.success= true;
			res.result=1;
			return true;
		}
		if (req.param1==4)
		{
			float force = req.param3-i_nothingPWM4;
			if (force>=0)
				force=(o_maxPWM4-o_nothingPWM4)*force/(i_maxPWM4-i_nothingPWM4);
			else
				force=(o_minPWM4-o_nothingPWM4)*force/(i_minPWM4-i_nothingPWM4);
			rightPWM=force;
			
			// propulsor
			sensor_msgs::JointState msg;
			msg.header.seq = idDir;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id="world";
			msg.name.push_back(diffboatLeftLinkName);
			msg.name.push_back(diffboatRightLinkName);
			msg.position.push_back(leftPWM);
			msg.position.push_back(rightPWM);
			thruster_command_pub.publish(msg);
			res.success= true;
			res.result=1;
			return true;
		}
	}
	res.success= false;
  	res.result=0;	
  	return false;
}


void stateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	msg_boatLocalPose.header=msg->header;
	msg_boatLocalPose.header.frame_id="map";
	msg_boatLocalPose.pose = msg->pose.pose;
//	compass_hdg = msg->pose.pose.orientation.
			
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

	// the tf::Quaternion has a method to acess roll pitch and yaw
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	yaw = yaw * 180.0 / M_PI; // conversion to degrees
	if( yaw < 0 ) yaw += 360.0;
	compass_hdg.data = yaw;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wau_mode_guided_pwm");

	ros::NodeHandle n;
	serviceCommand = n.advertiseService(boatNamespace+"/mavros/command", receiveCommandLong);
	thruster_command_pub = n.advertise<sensor_msgs::JointState>(boatNamespace+"thruster_command", 1);
	joint_setpoint_pub = n.advertise<sensor_msgs::JointState>(boatNamespace+"joint_setpoint", 1);
	state_sub = n.subscribe(boatNamespace+"state", 1, stateCallback);
	
	local_position_pub = n.advertise<geometry_msgs::PoseStamped>(boatNamespace+"local_position/pose", 1);
	compass_pub = n.advertise<std_msgs::Float64>(boatNamespace+"global_position/compass_hdg", 1);
	
	
	ros::Rate loop_rate(3);
	//ros::spin();
	while (ros::ok())
	{
		local_position_pub.publish(msg_boatLocalPose);
		compass_pub.publish(compass_hdg);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 1;
}
