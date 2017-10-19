#ifndef LASERFILTER_H
#define LASERFILTER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

class LaserFilter{
private:
	const char* subTopic;
	const char* pubTopic;
	bool isAngleFiltered;
	bool isRangeFiltered;
	bool isReversed;
	double minAngle,maxAngle;
	double minRange,maxRange;
	ros::Subscriber scanSub;
	ros::Publisher scanPub;
	float INF;
public:
	LaserFilter(const char* sTopic,const char* pTopic);
	void setAngleFilter(double minAng,double maxAng);
	void setRangeFilter(double minRan,double maxRan);
	void reverse();
	void start();
private:
	void onRecScan(const sensor_msgs::LaserScan::ConstPtr& rawScan);

};

#endif
