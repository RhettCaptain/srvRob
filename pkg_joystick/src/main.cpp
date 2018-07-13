#include "Joystick.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_joystick");
//	Joystick* stick = new Joystick(0,6,1,7,6,7);	//ps3
	Joystick* stick = new Joystick(0,4,1,5,4,5);	//betop
//	Joystick* stick = new Joystick(0,4,1,5,5,4);	//gjw betop
	stick->setSpd(0.2,0.4,0.6,0.5,1.0,1.5);
	stick->setPubRate(50);
	stick->setEnable(false);
	stick->start();
}
