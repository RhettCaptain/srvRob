#ifndef WEBHANDLER_H
#define WEBHANDLER_H

#include "ros/ros.h"
#include "Socket.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "tinyxml2.h"
#include <string>
#include <cstring>
#include <tf/tf.h>
#include <sstream>

class WebHandler{
private:
	SocketClient sock;
	ros::NodeHandle nHandle;
	ros::Publisher goalPub;
	ros::Publisher pathPub;
	ros::Publisher motionPub;
	ros::Publisher initPosePub;
	ros::Publisher resetPub;
	ros::Subscriber poseSub;
	geometry_msgs::PoseStamped robotPose;

public:
	WebHandler(const char* ip="192.168.1.250",unsigned short int port=9999);

private:
	void onSubPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pubGoal();
	void motion();
	void updateMap();
	void getPose();
	void pubPath();
	void updateInitPose();
};


#endif
