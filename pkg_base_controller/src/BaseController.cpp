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


void BaseController::start(){
	ros::spin();
}
