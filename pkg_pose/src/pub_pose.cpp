#include "ros/ros.h"
#include "PoseHandler.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_pub_pose");
	PoseHandler ph;
	ph.start();
}
