#include "Joystick.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_joystick");
	Joystick* stick = new Joystick();
	stick->start();
}
