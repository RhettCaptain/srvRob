#include "WeiNuoController.h"
#include <iostream>

using std::cout;
using std::endl;

WeiNuoController::WeiNuoController(const char* motorPort,double pWheelRadius,double pWheelDis,double pGearRatio):BaseController(pWheelRadius,pWheelDis,pGearRatio){
	motorDir = FF;
	openMotor(motorPort);
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
	twist2RotateSpd(msg,&l,&r);
	move(l,r);
	cout << "left:" << l << " right:" << r << endl;
}

void WeiNuoController::start(){
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
	motor->setInMode('r');
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
/*	const int minSpd = 700;
	const int maxSpd = 5000;
	if(leftRotateSpd<minSpd && leftRotateSpd>0){
		leftRotateSpd = minSpd;
		cout << "min rotate spd is" << minSpd << endl;
	}
	else if(leftRotateSpd > maxSpd){
		leftRotateSpd = maxSpd;
		cout << "max rotate spd is" << maxSpd << endl;
	}
	if(rightRotateSpd<minSpd && rightRotateSpd>0){
		rightRotateSpd = 0;
		cout << "min rotate spd is" << minSpd << endl;
	}
	else if(rightRotateSpd > maxSpd){
		rightRotateSpd = maxSpd;
		cout << "max rotate spd is" << maxSpd << endl;
	}
	const int diffLimit = 300;
	if(motorDir == FF || motorDir == BB){
		if((leftRotateSpd - rightRotateSpd) > diffLimit){
			leftRotateSpd = rightRotateSpd + diffLimit;
			cout << "max diff limit is " << diffLimit << endl;
		}
		else if((rightRotateSpd - leftRotateSpd) > diffLimit){
			rightRotateSpd = leftRotateSpd + diffLimit;
			cout << "max diff limit is " << diffLimit << endl;
		}
	}*/
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

usleep(30*1000);
int ls,rs;
getRotateSpd(ls,rs);
	cout << "ls: " << ls << " rs: " << rs << endl;

}

bool WeiNuoController::getRotateSpd(int& leftRotateSpd,int& rightRotateSpd){
	//send query command
	vector<uchar> queryCmd(7);
	queryCmd[0] = 0x1b;
	queryCmd[1] = 0x20;
	queryCmd[2] = 0x01;
	queryCmd[3] = 0x00;
	crc16Modbus(&queryCmd[1],3,&queryCmd[4],&queryCmd[5]);
	queryCmd[6] = 0x05;
	motor->writePort(queryCmd);
	
	//read data from hall sensor
	char ch=0xff;
	vector<uchar> hallBuffer(13);
	for(int i =0;i<13;i++){
		hallBuffer[i] = 0xff;
	}
	int errCount = 0;
	while(ch!=0x1b){
		motor->readPort(&ch,1);
		errCount++;
		if(errCount >=14){
			motor->flush();
			break;
		}
	}
	motor->readPort(hallBuffer,13);
	hallBuffer.insert(hallBuffer.begin(),ch);
for(int i=0;i<14;i++){printf("%x \n",hallBuffer[i]);}cout<<endl;	
	//analyse the speed
	leftRotateSpd = hallBuffer[3]<<24 + hallBuffer[4]<<16 + hallBuffer[5]<<8 + hallBuffer[6];
	rightRotateSpd = hallBuffer[7]<<24 + hallBuffer[8]<<16 + hallBuffer[9]<<8 + hallBuffer[10];

	//check validness
	if(hallBuffer[0] == 0x1b && hallBuffer[1] == 0x20 && hallBuffer[2] == 0x08 && hallBuffer[13] == 0x05){
		return true;
	}
	else{
		return false;
	}
}	
