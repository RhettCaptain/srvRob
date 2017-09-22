#ifndef POSEHANDLER_H
#define POSEHANDLER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

class PoseHandler{
private:
	ros::NodeHandle nh;
	ros::Subscriber slamPoseSub;
	ros::Publisher robotPosePub;
	geometry_msgs::PoseStamped robotPose;
public:
	PoseHandler();
	void start();
private: 
	void onRecSlamPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

#endif
