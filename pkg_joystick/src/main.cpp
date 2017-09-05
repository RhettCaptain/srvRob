#include "Joystick.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_joystick");
	Joystick* stick = new Joystick(0,6,1,7,6,7);
	stick->setSpd(0.1,0.2,0.3,0.2,0.4,0.6);
	stick->setPubRate(50);
	stick->start();
}
