#include "ObsSensor.h"


ObsSensor::ObsSensor(const char* pPortName,const char* pubTopic):minDis(0),maxDis(10){
	port = new SerialPort(pPortName);
	pub = nHandle.advertise<std_msgs::Bool>(pubTopic,10);
	for(int i=0;i<8;i++){
		disThreshold[i] = minDis;
	}
}

ObsSensor::ObsSensor(const ObsSensor& os):minDis(0),maxDis(10){
	port = new SerialPort("");
	*port = *(os.port);
	pub = os.pub;
	for(int i=0;i<8;i++){
		disThreshold[i] = os.disThreshold[i];
	}
}

const ObsSensor& ObsSensor::operator=(const ObsSensor& os){
	*port = *(os.port);
	pub = os.pub;
	for(int i=0;i<8;i++){
		disThreshold[i] = os.disThreshold[i];
	}
}

ObsSensor::~ObsSensor(){
	port->closePort();
	delete port;
}

void ObsSensor::setThreshold(double thr1,double thr2,double thr3,double thr4,double thr5,double thr6,double thr7,double thr8){
	disThreshold[0] = thr1;
	disThreshold[1] = thr2;
	disThreshold[2] = thr3;
	disThreshold[3] = thr4;
	disThreshold[4] = thr5;
	disThreshold[5] = thr6;
	disThreshold[6] = thr7;
	disThreshold[7] = thr8;
}

void ObsSensor::start(int pBaud,int pDataBits,int pStopBits,char pParity){
	port->openPort();
	port->setPort(pBaud,pDataBits,pStopBits,pParity);
//	port->setBlock(false);
//	port->setInMode('r');
//	port->setOutMode('r');
	pubObstacleState();
}

void ObsSensor::pubObstacleState(){
	while(ros::ok()){
		//read and analyse data
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
				//debug print
				for(int i=0;i<18;i+=2){
					printf("%d ",data[i]*256+data[i+1]);
				}
				printf("\n");
				//pub information
				std_msgs::Bool obsExist;
				obsExist.data = false;
				for(int i=0;i<8;i++){
					double dis = (data[i*2]*256+data[i*2+1])/100;		//xx m
					if(dis < disThreshold[i]){
						obsExist.data = true;
						break;
					}

				}
				pub.publish(obsExist);	
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
