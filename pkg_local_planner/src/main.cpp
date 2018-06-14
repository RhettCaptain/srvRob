#include "LocalPlanner.h"

int main(int argc, char** argv){
	ros::init(argc,argv,"node_local_planner");
 	LocalPlanner localPlanner;
	localPlanner.setRate(50);
	localPlanner.start();
}
