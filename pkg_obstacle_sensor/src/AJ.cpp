#include "AJ.h"

AJ::AJ(const char* pPortName,const char* pubTopic){
	port = new SerialPort(pPortName);
	pub = nHandle.advertise<std_msgs::Int16>(pubTopic,10);
}

AJ::AJ(const AJ& aj){
	port = new SerialPort("");
	*port = *(aj.port);
	pub = aj.pub;
}

const AJ& AJ::operator=(const AJ& aj){
	*port = *(aj.port);
	pub = aj.pub;
}

AJ::~AJ(){
	port->closePort();
	delete port;
}

void AJ::start(int pBaud,int pDataBits,int pStopBits,char pParity){
	port->openPort();
	port->setPort(pBaud,pDataBits,pStopBits,pParity);
	port->setBlock(true);
	port->setInMode('r');
	port->setOutMode('r');	
	if(port->isOpen()){
		analyse();
	}	
}

void AJ::analyse(){
	char tmp=0x00;
	char data[3];
	while(ros::ok()){
		while(ros::ok() && tmp!=-1){
			while(ros::ok() && port->readPort(&tmp,1)<1){}
		}
		while(ros::ok() && port->readPort(&tmp,1)<1){}
		data[0]=tmp;				
		while(ros::ok() && port->readPort(&tmp,1)<1){}
		data[1]=tmp;				
		while(ros::ok() && port->readPort(&tmp,1)<1){}
		data[2]=tmp;				
		if(data[2] == (data[0]+data[1])&0x00ff){
			printf("e:%d\n",data[0]*256+(uchar)data[1]);
			std_msgs::Int16 dis;
			dis.data = data[0]*256+(uchar)data[1];
			pub.publish(dis);
		}
		tmp=0x00;
	}
}

