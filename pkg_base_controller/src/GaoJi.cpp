#include "GaoJi.h"
#include <iostream>

using std::cout;
using std::endl;

GaoJi::GaoJi(const char* motorPort,double pWheelRadius,double pWheelDis,double pGearRatio):BaseController(pWheelRadius,pWheelDis,pGearRatio){
	motorDir = FF;
	openMotor(motorPort);
	pub = nHandle.advertise<nav_msgs::Odometry>("topic_odometer_sensor",10);
	pubTime = ros::Time::now();
}

GaoJi::GaoJi(const GaoJi& wn):BaseController(getWheelRadius(),getWheelDis(),getGearRatio()){
	motor = new SerialPort("");
	*motor = *(wn.motor);
	motorDir = wn.motorDir;
}

GaoJi& GaoJi::operator=(const GaoJi& wn){
	*motor = *(wn.motor);
	motorDir = wn.motorDir;
	return *this;
}

void GaoJi::onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	double l,r;
	twist2RotateSpd(msg,&l,&r);
	move(l,r);
	pubFakeOdometerMsg(msg);	//it's a fake msg using open-loop and integration
}

void GaoJi::start(){
	subCmdVel();
	leftMove(0);
	rightMove(0);
	setLeftBackTime(1000);
	setRightBackTime(1000);
	enable(true);
	ros::Rate loop(100);
	while(ros::ok()){
		ros::spinOnce();
	//	pubOdometerMsg();
		loop.sleep();
	}

}

void GaoJi::openMotor(const char* motorPort,int baud,int dataBits,int stopBits,char parity){
	motor = new SerialPort(motorPort);
	motor->openPort();
	motor->setPort(baud,dataBits,stopBits,parity);
	motor->setBlock(false);
	motor->setInMode('r');
}

void GaoJi::closeMotor(){
	if(motor!=NULL && motor->isOpen()){
		motor->closePort();
	}
}

void GaoJi::crc16Modbus(uchar *p, int len,uchar* hCrc,uchar* lCrc)
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

void GaoJi::move(int leftRotateSpd,int rightRotateSpd){
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
		leftMove(0);
		rightMove(0);
		usleep(200*1000);
	}
	motorDir = newDir;	//update direction
	
	if(leftRotateSpd < 0){
		leftBack(false);
	}else{
		leftBack(true);
	}
	if(rightRotateSpd < 0){
		rightBack(false);
	}else{
		rightBack(true);
	}

	//send the cmd to motor driver
	leftRotateSpd = abs(leftRotateSpd);
	rightRotateSpd = abs(rightRotateSpd);

	leftMove(leftRotateSpd);
	rightMove(rightRotateSpd);

//for(uchar c:moveCmd){
//  std::cout <<(int)c << " " ;
//}
}


void GaoJi::pubFakeOdometerMsg(const geometry_msgs::Twist::ConstPtr& msg){
	//integrate the spd to dis
	ros::Time newPubTime = ros::Time::now();
	double deltaTime = (newPubTime - pubTime).toSec();
	pubTime = newPubTime; 	//update the odometer msg publish time
	double linDis = msg->linear.x * deltaTime;
	double angDis = msg->angular.z * deltaTime;
//std::cout << "deltaTime: " << deltaTime << " angDis: " << msg->angular.z << std::endl;
	//get twist
	//trust the expected twist
		
	//publish infomation 
	nav_msgs::Odometry odometerMsg;
	odometerMsg.header.stamp = pubTime;
	odometerMsg.header.frame_id = "base_link";
	odometerMsg.pose.pose.position.x = linDis;
	odometerMsg.pose.pose.position.y = 0;
	odometerMsg.pose.pose.position.z = 0;
	odometerMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(angDis);
	odometerMsg.twist.twist = *msg;
	//pub
	pub.publish(odometerMsg);
	pubTime = newPubTime;
}

void GaoJi::leftMove(int rotateSpd){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x06;
	moveCmd[2] = 0x10;
	moveCmd[3] = 0x01;
	moveCmd[4] = (uchar)(rotateSpd>>8);
	moveCmd[5] = (uchar)(rotateSpd & 0x00ff);
	crc16Modbus(&moveCmd[0],6,&moveCmd[7],&moveCmd[6]);
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}

void GaoJi::rightMove(int rotateSpd){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x06;
	moveCmd[2] = 0x10;
	moveCmd[3] = 0x02;
	moveCmd[4] = (uchar)(rotateSpd>>8);
	moveCmd[5] = (uchar)(rotateSpd & 0x00ff);
	crc16Modbus(&moveCmd[0],6,&moveCmd[7],&moveCmd[6]);
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}

void GaoJi::setLeftBackTime(int t){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x06;
	moveCmd[2] = 0x10;
	moveCmd[3] = 0x05;
	moveCmd[4] = (uchar)(t>>8);
	moveCmd[5] = (uchar)(t & 0x00ff);
	crc16Modbus(&moveCmd[0],6,&moveCmd[7],&moveCmd[6]);
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}

void GaoJi::setRightBackTime(int t){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x06;
	moveCmd[2] = 0x10;
	moveCmd[3] = 0x06;
	moveCmd[4] = (uchar)(t>>8);
	moveCmd[5] = (uchar)(t & 0x00ff);
	crc16Modbus(&moveCmd[0],6,&moveCmd[7],&moveCmd[6]);
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}

void GaoJi::leftBack(bool on){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x05;
	moveCmd[2] = 0x08;
	moveCmd[3] = 0x01;
	if(on){
		moveCmd[4] = 0xff;
		moveCmd[6] = 0xdf;
		moveCmd[7] = 0x9a;
	}else{
		moveCmd[4] = 0x00;
		moveCmd[6] = 0x9e;
		moveCmd[7] = 0x6a;
	}
	moveCmd[5] = 0x00;
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}

void GaoJi::rightBack(bool on){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x05;
	moveCmd[2] = 0x08;
	moveCmd[3] = 0x02;
	if(on){
		moveCmd[4] = 0xff;
		moveCmd[6] = 0x2f;
		moveCmd[7] = 0x9a;
	}else{
		moveCmd[4] = 0x00;
		moveCmd[6] = 0x6e;
		moveCmd[7] = 0x6a;
	}
	moveCmd[5] = 0x00;
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}

void GaoJi::enable(bool on){
	vector<uchar> moveCmd(8);
	moveCmd[0] = 0x01;
	moveCmd[1] = 0x05;
	moveCmd[2] = 0x08;
	moveCmd[3] = 0x00;
	if(on){
		moveCmd[4] = 0xff;
		moveCmd[6] = 0x8e;
		moveCmd[7] = 0x5a;
	}else{
		moveCmd[4] = 0x00;
		moveCmd[6] = 0xcf;
		moveCmd[7] = 0xaa;
	}
	moveCmd[5] = 0x00;
	motor->writePort(moveCmd);	//send cmd
	usleep(50*1000);	
}
