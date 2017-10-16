#ifndef OBSSENSOR_H
#define OBSSENSOR_H

#include "ros/ros.h"
#include "SerialPort.h"
#include "std_msgs/Bool.h"


class ObsSensor{
private:
	SerialPort* port;
	ros::NodeHandle nHandle;
	ros::Publisher pub;
	double disThreshold[8];
	const double minDis,maxDis;
public:
	ObsSensor(const char* pPortName,const char* pubToic="topic_obstacle");
	ObsSensor(const ObsSensor& os);
	const ObsSensor& operator=(const ObsSensor& os);
	~ObsSensor();
	void setThreshold(double thr1=0,double thr2=0,double thr3=0,double thr4=0,double thr5=0,double thr6=0,double thr7=0,double thr8=0);
	void start(int pBaud=115200,int pDataBits=8,int pStopBits=1,char pParity='n');	

private:
	void pubObstacleState();
};

#endif
