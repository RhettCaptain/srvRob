#include "EAIController.h"
#include <iostream>

using std::cout;
using std::endl;

EAIController::EAIController(){
}

void EAIController::onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	cout << "i;m eai" << endl;
}

void EAIController::start(){
	ros::Rate loop(10);
	while(ros::ok()){
		ros::spinOnce();
		loop.sleep();
	}

}
