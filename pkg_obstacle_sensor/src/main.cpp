#include "ObsSensor.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_obstacle_sensor");
	ObsSensor* obsSensor = new ObsSensor("/dev/ttyUSB2");
	obsSensor->start();
}
