#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include "tf/transform_broadcaster.h"

class Odometry{
private:
	ros::NodeHandle nHandle;
	ros::Subscriber odometerSub;
	ros::Publisher odometryPub;
	tf::TransformBroadcaster tfBroadcaster;

	geometry_msgs::Pose odometryPose;
	geometry_msgs::Twist odometryTwist;
public:
	Odometry(double x=0,double y=0,double th=0);
	Odometry(const geometry_msgs::Pose& initPose);
	void start(int spinRate=50);
	void pubOdom();
	void broadcastTf();
private:
	void onRecOdometerMsg(const nav_msgs::Odometry::ConstPtr& msg);
	void normalizeAng(double& ang);
};

#endif
