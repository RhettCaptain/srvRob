#include "WeiNuoController.h"
#include <iostream>

using std::cout;
using std::endl;

WeiNuoController::WeiNuoController(double pWheelRadius,double pWheelDis,double pGearRatio):BaseController(pWheelRadius,pWheelDis,pGearRatio){
	
}

WeiNuoController::WeiNuoController(const WeiNuoController& wn):BaseController(getWheelRadius(),getWheelDis(),getGearRatio()){
	*motor = *(wn.motor);
}

WeiNuoController& WeiNuoController::operator=(const WeiNuoController& wn){
	*motor = *(wn.motor);
	return *this;
}
void WeiNuoController::onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	double l,r;
	getRotateSpd(msg,&l,&r);
	cout << "left:" << l << " right:" << r << endl;
}

void WeiNuoController::start(){
	subCmdVel();
	ros::Rate loop(10);
	while(ros::ok()){
		ros::spinOnce();
		loop.sleep();
	}

}

void WeiNuoController::openMotor(const char* motorPort,int baud,int dataBits,int stopBits,char parity){
	motor = new SerialPort(motorPort);
	motor->openPort();
	motor->setPort(baud,dataBits,stopBits,parity);
	motor->setBlock(false);
}

void WeiNuoController::closeMotor(){
	if(motor!=NULL && motor->isOpen()){
		motor->closePort();
	}
}
