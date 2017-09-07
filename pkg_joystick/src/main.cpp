#include "Joystick.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_joystick");
//	Joystick* stick = new Joystick(0,6,1,7,6,7);	//ps3
	Joystick* stick = new Joystick(0,4,1,5,4,5);	//betop
	stick->setSpd(0.25,0.3,0.33,0.3,0.6,0.9);
	stick->setPubRate(50);
	stick->start();
}
