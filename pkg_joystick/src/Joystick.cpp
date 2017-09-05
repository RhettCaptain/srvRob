#include "Joystick.h"

using namespace std;

Joystick::Joystick(){
	sub = nHandle.subscribe("joy",10,&Joystick::updateState,this);
	state = STOP;
	speed = NOR_SPD;
}


void Joystick::updateState(const sensor_msgs::Joy::ConstPtr& msg){
	bool isForward = false;
	bool isBack = false;
	bool isLeft = false;
	bool isRight = false;
	bool isSpdDown = false;
	bool isSpdUp = false;
	//analyse the operation
	//is left or right
	if(msg->axes[0]>0 || msg->axes[6]>0){
		isLeft = true;
	}
	else if(msg->axes[0]<0 || msg->axes[6]<0){
		isRight = true;
	}
	//is forward or back
	if(msg->axes[1]>0 || msg->axes[7]>0){
		isForward = true;
	}
	else if(msg->axes[1]<0 || msg->axes[7]<0){
		isBack = true;
	}
	//is speed down or up
	if(msg->buttons[6] == 1){
		isSpdDown = true;
	}
	else if(msg->buttons[7] == 1){
		isSpdUp = true;
	}
	//update state and speed
	//update state
	if(isLeft){
		if(isForward){
			state = FOR_LEFT;
		}
		else if(isBack){
			state = BACK_LEFT;
		}
		else{
			state = LEFT;
		}
	}
	else if(isRight){
		if(isForward){
			state = FOR_RIGHT;
		}
		else if(isBack){
			state = BACK_RIGHT;
		}
		else{
			state = RIGHT;
		}
	}
	else if(isForward){
		state = FORWARD;
	}
	else if(isBack){
		state = BACK;
	}
	else{
		state = STOP;
	}
	//update speed
	if(isSpdDown){
		speed = LOW_SPD;
	}
	else if(isSpdUp){
		speed = HIGH_SPD;
	}
	else{
		speed = NOR_SPD;
	}
}

void Joystick::sendCmdVel(){
	switch(speed){
	case HIGH_SPD:
		cout << "high spd ";
		break;
	case LOW_SPD:
		cout << "low spd  ";
		break;
	case NOR_SPD:
		cout << "nor spd  ";
		break;
	default:
		break;
	}
	switch(state){
	case FORWARD:
		cout << "forward  ";
		break;
	case BACK:
		cout << "back  ";
		break;
	case LEFT:
		cout << "left  ";
		break;
	case RIGHT:
		cout << "right  ";
		break;
	case FOR_LEFT:
		cout << "for_left  ";
		break;
	case FOR_RIGHT:
		cout << "for_right  ";
		break;
	case BACK_LEFT:
		cout << "back_left  ";
		break;
	case BACK_RIGHT:
		cout << "back_right  ";
		break;
	case STOP:
		cout << "stop  ";
		break;
	default:
		break;
	}
	cout << endl;
}

void Joystick::start(){
	ros::Rate loopRate(100);
	while(ros::ok()){
		ros::spinOnce();
		sendCmdVel();
		loopRate.sleep();
	}
}
