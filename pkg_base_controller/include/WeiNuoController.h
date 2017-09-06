#ifndef WEINUOCONTROLLER_H
#define WEINUOCONTROLLER_H

#include "BaseController.h"
#include "SerialPort.h"

class WeiNuoController:public BaseController{
private:
	SerialPort* motor;
	
	enum MotorDir{FF=0x11,FB=0x10,BF=0x01,BB=0x00}motorDir;
	void crc16Modbus(uchar *p, int len,uchar* hCrc,uchar* lCrc);
	void openMotor(const char* motorPort,int baud=115200,int dataBits=8,int stopBits=1,char parity='n');
	void closeMotor();
	void move(int leftRotateSpd,int rightRotateSpd);	
public:
	WeiNuoController(double pWheelRadius,double pWheelDis,double pGearRatio);
	WeiNuoController(const WeiNuoController& wn);
	WeiNuoController& operator=(const WeiNuoController& wn);
	void onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
	void start(const char* motorPort="/dev/ttyS0");

	
};


#endif
