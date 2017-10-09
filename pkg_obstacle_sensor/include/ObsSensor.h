#ifndef OBSSENSOR_H
#define OBSSENSOR_H

#include "ros/ros.h"
#include "SerialPort.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
#include <vector>

class ObsSensor{
private:
	SerialPort* port;
	ros::NodeHandle nHandle;
	ros::Publisher pub;
	bool onOff[8];
	geometry_msgs::Pose sensorPose[8];
	int chosenCount;
	double disThreshold;
public:
	ObsSensor(const char* pPortName,const char* pubToic="topic_obstacles_pose");
	ObsSensor(const ObsSensor& os);
	const ObsSensor& operator=(const ObsSensor& os);
	~ObsSensor();
	void chooseSensor(unsigned char pOnOff);
	void setSensorPose(int idx,const geometry_msgs::Pose& pPose);
	void setSensorPose(int idx,const double x,const double y,const double th);
	void setThreshold(double dis);
	void start(int pBaud=115200,int pDataBits=8,int pStopBits=1,char pParity='n');	

private:
	void pubObstaclesPose();
};

#endif
