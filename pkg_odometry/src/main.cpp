#include "Odometry.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_odometry");
	Odometry odometry;
	odometry.start(50);
}
