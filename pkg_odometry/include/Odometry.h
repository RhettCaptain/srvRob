#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

class Odometry{
private:
	ros::NodeHandle nHandle;
	ros::Subscriber odometerSub;
	ros::Publisher odometryPub;

	geometry_msgs::Pose odometryPose;
public:
	void Odometry(double x=0,double y=0,double th=0);
	void Odometry(const geometry_msgs::Pose& initPose);

private:
	void onRecOdometerMsg(const nav_msgs::Odometry::ConstPtr& msg);
	
}

#endif
