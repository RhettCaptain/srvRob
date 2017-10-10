#ifndef WEBHANDLER_H
#define WEBHANDLER_H

#include "ros/ros.h"
#include "Socket.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include <cstring>
#include <tf/tf.h>

class WebHandler{
private:
	SocketClient sock;
	ros::NodeHandle nHandle;
	ros::Publisher goalPub;

public:
	WebHandler(const char* ip="192.168.1.250",unsigned short int port=9999);

private:
	void pubGoal();
	void motion();
};


#endif
