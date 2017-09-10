#include "BaseController.h"

using std::string;

BaseController::BaseController(double pWheelRadius,double pWheelDis,double pGearRatio){
	wheelRadius = pWheelRadius;
	wheelDis = pWheelDis;
	gearRatio = pGearRatio;
}

void BaseController::subCmdVel(string topic){
	cmdVelSub = nHandle.subscribe(topic,10,&BaseController::onRecCmdVel,this);
}

void BaseController::subCmdVel(char* topic){
	cmdVelSub = nHandle.subscribe(topic,10,&BaseController::onRecCmdVel,this);
}
/*
void BaseController::setWheelRadius(double rad){
	wheelRadius = rad;
}
*/
/*
void BaseController::setWheelDis(double dis){
	wheelDis = dis;
}
*/
/*
void BaseController::setGearRatio(double rat){
	gearRatio = rat;
}
*/

double BaseController::getWheelRadius(){
	return wheelRadius;
}

double BaseController::getWheelDis(){
	return wheelDis;
}

double BaseController::getGearRatio(){
	return gearRatio;
}

void BaseController::twist2RotateSpd(const geometry_msgs::Twist::ConstPtr& msg,double* leftRotateSpd,double* rightRotateSpd,double leftFixFactor,double rightFixFactor){
	double linSpd = msg->linear.x;
	double angSpd = msg->angular.z;
	double leftLinSpd = linSpd - angSpd * wheelDis / 2;
	double rightLinSpd = linSpd + angSpd * wheelDis / 2;
	*leftRotateSpd = leftLinSpd * gearRatio * 30 / M_PI / wheelRadius * leftFixFactor; 
	*rightRotateSpd = rightLinSpd * gearRatio * 30 / M_PI / wheelRadius * rightFixFactor; 
}

void BaseController::rotateSpd2Twist(double& linSpd,double& angSpd,const double leftRotateSpd,const double rightRotateSpd,double leftFixFactor,double rightFixFactor){
	double leftLinSpd = leftRotateSpd / gearRatio / 30 * M_PI * wheelRadius * leftFixFactor;
	double rightLinSpd = rightRotateSpd / gearRatio /30 * M_PI * wheelRadius * rightFixFactor;
	linSpd = (leftLinSpd + rightLinSpd) / 2;
	angSpd = (rightLinSpd - leftLinSpd) / wheelDis; 
}

void BaseController::rotateSpd2Twist(geometry_msgs::Twist& msg,const double leftRotateSpd,const double rightRotateSpd,double leftFixFactor,double rightFixFactor){
	double leftLinSpd = leftRotateSpd / gearRatio / 30 * M_PI * wheelRadius * leftFixFactor;
	double rightLinSpd = rightRotateSpd / gearRatio /30 * M_PI * wheelRadius * rightFixFactor;
	double linSpd = (leftLinSpd + rightLinSpd) / 2;
	double angSpd = (rightLinSpd - leftLinSpd) / wheelDis; 
	msg.linear.x = linSpd;
	msg.linear.y = 0.0;
	msg.linear.z = 0.0;
	msg.angular.x = 0.0;
	msg.angular.y = 0.0;
	msg.angular.z = angSpd;
}

double BaseController::rotateRound2Dis(double round){
	double dis = round / gearRatio * M_PI * wheelRadius / 30;
	return dis;
}

void BaseController::dis2Shift(double& linDis,double& angDis,const double leftDis,const double rightDis){
	angDis = (rightDis - leftDis) / wheelDis;
	if(angDis == 0){
		linDis = leftDis;
	}
	else{
		linDis = (leftDis + rightDis) / angDis * sin(angDis/2);
	}
}

void BaseController::start(){
	ros::spin();
}
