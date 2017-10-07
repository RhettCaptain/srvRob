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
//	port->setBlock(true);
//	port->setInMode('r');
//	port->setOutMode('r');
	pubObstaclesPose();
}

void ObsSensor::pubObstaclesPose(){
std::cout << std::hex;
	while(ros::ok()){
		char tmp=0xFF;
		int nCount;
		int n = port->readPort(&tmp,1);
		if(n>0){
	//		std::cout << (unsigned int)((unsigned char)tmp) << " ";
		printf("%02x ",(unsigned int)((unsigned char)tmp));
		}
/*
		while((unsigned int)((unsigned char)tmp)!=0xbb){
			nCount = port->readPort(&tmp,1);
//std::cout << "debug" << std::endl;
std::cout << std::dec;
if(nCount >0) std::cout << "drop: " <<(unsigned int)((unsigned char)tmp) << std::endl;
		}
		nCount = port->readPort(&tmp,1);
		if(tmp == 0x01){
			nCount = port->readPort(&tmp,1);
			if(tmp == 0x12){
char ttt[18];
nCount = port->readPort(ttt,18);
for(int i=0;i<nCount;i++){
	std::cout << std::dec;
	std::cout << (unsigned int)((unsigned char)ttt[i]) << " ";
}
std::cout << std::endl;
				//valid seq
				vector<uchar> data(18);
				port->readPort(data,18);
				for(int i=0;i<18;i++){
					std::cout << std::hex;
					std::cout << (int)data[i] << std::endl;
				}
			}
			else{
				continue;	
			}
		}
		else{
			continue;
		}
*/
	}
}
