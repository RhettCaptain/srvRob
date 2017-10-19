#include "LaserFilter.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_laser_filter");
	LaserFilter* laserFilter = new LaserFilter("topic_raw_scan","scan");
	double minAng = -1.57;
	double maxAng = 1.57;
	double minRan = 0.72;
	double maxRan = 5.01;
//	laserFilter->setAngleFilter(minAng,maxAng);
//	laserFilter->setRangeFilter(minRan,maxRan);
//	laserFilter->reverse();
	laserFilter->start();
}
