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
	BaseController(double pWheelRadius,double pWheelDis,double pGearRatio);
	void subCmdVel(std::string topic="cmd_vel");
	void subCmdVel(char* topic);

//	void setWheelRadius(double rad);
//	void setWheelDis(double dis);
//	void setGearRatio(double rat);

	double getWheelRadius();
	double getWheelDis();
	double getGearRatio();
	
	void twist2RotateSpd(const geometry_msgs::Twist::ConstPtr& msg,double* leftSpd,double* rightSpd,double leftFixFactor=1.0,double rightFixFactor=1.0);
	void rotateSpd2Twist(double& linSpd,double& angSpd,const double leftRotateSpd,const double rightRotateSpd,double leftFixFactor=1.0,double rightFixFactor=1.0);
	void rotateSpd2Twist(geometry_msgs::Twist& msg,const double leftRotateSpd,const double rightRotateSpd,double leftFixFactor=1.0,double rightFixFactor=1.0);
	//get the distance one wheel move according the round motor rotates
	double rotateRound2Dis(double round);
	//get the linear distance and angular shift from two wheels' distance 
	void dis2Shift(double& linDis,double& angDis,const double leftDis,const double rightDis);
	virtual void onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg)=0;
	virtual void start();
};

#endif
