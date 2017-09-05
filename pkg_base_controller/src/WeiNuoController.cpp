#include "WeiNuoController.h"
#include <iostream>

using std::cout;
using std::endl;

WeiNuoController::WeiNuoController(){
}

void WeiNuoController::onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
	double l,r;
	getRotateSpd(msg,&l,&r);
	cout << "left:" << l << " right:" << r << endl;
}

void WeiNuoController::start(){
	ros::Rate loop(10);
	while(ros::ok()){
		ros::spinOnce();
		loop.sleep();
	}

}
