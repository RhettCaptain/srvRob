#ifndef WEINUOCONTROLLER_H
#define WEINUOCONTROLLER_H

#include "BaseController.h"
#include "SerialPort.h"

class WeiNuoController:public BaseController{
private:
	SerialPort* motor;
	
//	void move();	
public:
	WeiNuoController(double pWheelRadius,double pWheelDis,double pGearRatio);
	WeiNuoController(const WeiNuoController& wn);
	WeiNuoController& operator=(const WeiNuoController& wn);
	void onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
	void start();

	void setMotorPort(const char* motorPort);
	
};


#endif
