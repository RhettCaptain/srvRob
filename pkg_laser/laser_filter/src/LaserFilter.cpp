#include "LaserFilter.h"


LaserFilter::LaserFilter(const char*sTopic,const char* pTopic):subTopic(sTopic),pubTopic(pTopic){
	ros::NodeHandle nHandle;
	scanSub = nHandle.subscribe(subTopic,10,&LaserFilter::onRecScan,this);
	scanPub = nHandle.advertise<sensor_msgs::LaserScan>(pubTopic,10);
	isAngleFiltered = false;
	isRangeFiltered = false;
	isReversed = false;
	minAngle = 0;
	maxAngle = 0;
	minRange = 0;
	maxRange = 0;
	INF = 0;
}

void LaserFilter::setAngleFilter(double minAng,double maxAng){
	minAngle = minAng;
	maxAngle = maxAng;
	isAngleFiltered = true;
}

void LaserFilter::setRangeFilter(double minRan,double maxRan){
	minRange = minRan;
	maxRange = maxRan;
	isRangeFiltered = true;
	INF = maxRange+1;
}

void LaserFilter::reverse(){
	isReversed = true;
}

void LaserFilter::start(){
	ros::spin();
}

void LaserFilter::onRecScan(const sensor_msgs::LaserScan::ConstPtr& rawScan){
	sensor_msgs::LaserScan filScan = *rawScan;
	if(isAngleFiltered){
		int startIdx = (minAngle - filScan.angle_min)/filScan.angle_increment;
		int endIdx = (maxAngle - filScan.angle_min)/filScan.angle_increment;
		int newLen = endIdx - startIdx + 1;
		filScan.ranges.resize(newLen);
		filScan.intensities.resize(newLen);
		for(int i=0;i<newLen;i++){
			filScan.ranges[i] = rawScan->ranges[startIdx+i];
			filScan.intensities[i] = rawScan->intensities[startIdx+i];
		}
		filScan.angle_min = minAngle;
		filScan.angle_max = maxAngle;
	}	
	if(isRangeFiltered){
		filScan.range_min = minRange;
		filScan.range_max = maxRange;
	}
	if(isReversed){
		filScan.angle_increment = -filScan.angle_increment;
	}
	scanPub.publish(filScan);
}
