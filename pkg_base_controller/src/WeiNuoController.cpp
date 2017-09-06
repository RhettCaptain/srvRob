#include "WeiNuoController.h"
#include <iostream>

using std::cout;
using std::endl;

WeiNuoController::WeiNuoController(double pWheelRadius,double pWheelDis,double pGearRatio):BaseController(pWheelRadius,pWheelDis,pGearRatio){
	motorDir = FF;
}

WeiNuoController::WeiNuoController(const WeiNuoController& wn):BaseController(getWheelRadius(),getWheelDis(),getGearRatio()){
	*motor = *(wn.motor);
	motorDir = wn.motorDir;
}

WeiNuoController& WeiNuoController::operator=(const WeiNuoController& wn){
	*motor = *(wn.motor);
	motorDir = wn.motorDir;
	return *this;
}
void WeiNuoController::onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	double l,r;
	getRotateSpd(msg,&l,&r);
	move(l,r);
	cout << "left:" << l << " right:" << r << endl;
}

void WeiNuoController::start(const char* motorPort){
	if(motor==NULL || !motor->isOpen()){
		openMotor(motorPort);
	}
	subCmdVel();
	ros::Rate loop(100);
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


void WeiNuoController::crc16Modbus(uchar *p, int len,uchar* hCrc,uchar* lCrc)
{
	char i;
	int j;
	unsigned int crc=0xffff;
	
	for(j=0;j<len;j++)
	{
		crc^=(*p);
		p++;
		for(i=8;i!=0;i--)
		{
			if(crc&1)
			{
				crc>>=1;
				crc^=0xa001;
			}
			else
			{
				crc>>=1;
			}
		}
	}
	*hCrc = (crc&0xff00)>>8;
	*lCrc = (crc&0x00ff); 
}

void WeiNuoController::move(int leftRotateSpd,int rightRotateSpd){
	//update motor direction
	MotorDir newDir= FF;
	if(leftRotateSpd>=0){
		if(rightRotateSpd>=0){
			newDir = FF;
		}
		else{
			newDir = FB;
		}
	}
	else{
		if(rightRotateSpd>=0){
			newDir = BF;
		}
		else{
			newDir = BB;
		}
	}
	//if direction is changed, 1s delay is needed in case of stop
	if(newDir != motorDir){
		usleep(1000*1000);
	}
	motorDir = newDir;	//update direction

	//send the cmd to motor driver
	leftRotateSpd = abs(leftRotateSpd);
	rightRotateSpd = abs(rightRotateSpd);
	if(leftRotateSpd<700){
		leftRotateSpd = 700;
		cout << "set spd as min rotate spd 700r/m" << endl;
	}
	else if(leftRotateSpd > 5000){
		leftRotateSpd = 5000;
		cout << "set spd as max rotate spd 5000r/m" << endl;
	}
	vector<uchar> moveCmd(9);
	moveCmd[0] = 0x1b;
	moveCmd[1] = motorDir^0x10;
	moveCmd[2] = (uchar)(leftRotateSpd>>8);
	moveCmd[3] = (uchar)(leftRotateSpd & 0x00ff);
	moveCmd[4] = (uchar)(rightRotateSpd>>8);
	moveCmd[5] = (uchar)(rightRotateSpd & 0x00ff);
	crc16Modbus(&moveCmd[1],5,&moveCmd[6],&moveCmd[7]);
	moveCmd[8] = 0x05;
	motor->writePort(moveCmd);	//send cmd
}
