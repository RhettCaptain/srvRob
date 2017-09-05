#ifndef BASECONTROLLER_H
#define BASECONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>

class BaseController{
private:
	ros::NodeHandle nHandle;
	ros::Subscriber cmdVelSub;

	double wheelRadius;
	double wheelDis;
	double gearRatio;

public:
	BaseController(double pWheelRadius=1,double pWheelDis=1,double pGearRatio=1);
	void subCmdVel(std::string topic="cmd_vel");
	void subCmdVel(char* topic);

	void setWheelRadius(double rad);
	void setWheelDis(double dis);
	void setGearRatio(double rat);

	void getRotateSpd(const geometry_msgs::Twist::ConstPtr& msg,double* leftSpd,double* rightSpd,double leftFixFactor=1.0,double rightFixFactor=1.0);
	virtual void onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg)=0;
	virtual void start();
};

#endif
