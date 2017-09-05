#ifndef BASECONTROLLER_H
#define BASECONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

using std::string;

class BaseController{
private:
	string topic_cmd_vel;
	ros::NodeHandle nHandle;
	ros::Subscriber cmdVelSub;
public:
	BaseController();
	void setTopicCmdVel(string topic);
	void setTopicCmdVel(char* topic);
	virtual void move()=0;
	virtual void start();
};

#endif
