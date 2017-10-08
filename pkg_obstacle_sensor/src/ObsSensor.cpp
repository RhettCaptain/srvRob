#include "ObsSensor.h"

using std::vector;

ObsSensor::ObsSensor(const char* pPortName,const char* pubTopic){
	port = new SerialPort(pPortName);
	pub = nHandle.advertise<nav_msgs::Path>(pubTopic,10);
	for(int i=0;i<8;i++){
		onOff[i] = false;
	}
	chosenCount = 0;
}

ObsSensor::ObsSensor(const ObsSensor& os){
	port = new SerialPort("");
	*port = *(os.port);
	pub = os.pub;
	for(int i=0;i<8;i++){
		onOff[i] = os.onOff[i];
		sensorPose[i] = os.sensorPose[i];
	}
	chosenCount = os.chosenCount;
}

const ObsSensor& ObsSensor::operator=(const ObsSensor& os){
	*port = *(os.port);
	pub = os.pub;
	for(int i=0;i<8;i++){
		onOff[i] = os.onOff[i];
		sensorPose[i] = os.sensorPose[i];
	}
	chosenCount = os.chosenCount;
}

ObsSensor::~ObsSensor(){
	port->closePort();
	delete port;
}

void ObsSensor::chooseSensor(unsigned char pOnOff){
	unsigned char flag = 0x80;
	for(int i=0;i<8;i++){
		if(pOnOff & (flag>>i)){
			onOff[i] = true;
			chosenCount++;
		}
	}
}

void ObsSensor::setSensorPose(int idx,const geometry_msgs::Pose& pPose){
	sensorPose[idx] = pPose;
}

void ObsSensor::setSensorPose(int idx,const double x,const double y,const double th){
	sensorPose[idx].position.x = x;
	sensorPose[idx].position.y = y;
	sensorPose[idx].orientation = tf::createQuaternionMsgFromYaw(th);
}

void ObsSensor::start(int pBaud,int pDataBits,int pStopBits,char pParity){
	port->openPort();
	port->setPort(pBaud,pDataBits,pStopBits,pParity);
//	port->setBlock(false);
//	port->setInMode('r');
//	port->setOutMode('r');
	pubObstaclesPose();
}

void ObsSensor::pubObstaclesPose(){
	while(ros::ok()){
		char tmp=0xFF;

		while(ros::ok() && (unsigned int)((unsigned char)tmp)!=0xbb){
			while(ros::ok() && port->readPort(&tmp,1) < 1){
			}
		}
		while(ros::ok() && port->readPort(&tmp,1) < 1){
		}
		if((unsigned int)((unsigned char)tmp) == 0x01){
			while(ros::ok() && port->readPort(&tmp,1) < 1){
			}
			if((unsigned int)((unsigned char)tmp) == 0x12){
				//valid seq
				vector<uchar> data(18);
				int nCount = -1;
				while(ros::ok() && nCount < 1){
					nCount = port->readPort(data,18);
				}
				for(int i=0;i<18-nCount;i++){
					while(ros::ok() && port->readPort(&tmp,1) < 1 ){
					}
					data[nCount+i] = tmp;
				}
				for(int i=0;i<18;i+=2){
					printf("%d ",data[i]*256+data[i+1]));
				}
				printf("\n");
			}
			else{
				continue;	
			}
		}
		else{
			continue;
		}

	}
}
