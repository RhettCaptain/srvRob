#ifndef AJ_H
#define AJ_H

#include "ros/ros.h"
#include "SerialPort.h"
#include <string>
#include "std_msgs/Int16.h"

class AJ{
private:
	SerialPort* port;
	ros::NodeHandle nHandle;
	ros::Publisher pub;

public:
	AJ(const char* pPortName,const char* pubTopic="topic_obstacle");
	AJ(const AJ& aj);
	const AJ& operator=(const AJ& aj);
	~AJ();
	void start(int pBaud=9600,int pDataBits=8,int pStopBits=1,char pParity='n');
private:
	void analyse();	
}; 

#endif
