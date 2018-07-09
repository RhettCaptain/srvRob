#ifndef POSEHANDLER_H
#define POSEHANDLER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "tf/tf.h"

class PoseHandler{
private:
	ros::NodeHandle nh;
	ros::Subscriber slamPoseSub;
	ros::Publisher robotPosePub;
	ros::Publisher resetPosePub;
	ros::Publisher resetCmdPub;
	ros::Publisher motionCmdPub;
	geometry_msgs::PoseStamped robotPose;
	double xErrLimit;
	double yErrLimit;
	double thErrLimit;
public:
	PoseHandler();
	void start();
private: 
	void onRecSlamPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

#endif
