#include "Joystick.h"

Joystick::Joystick(){
	joySub = nHandle.subscribe("joy",10,&Joystick::updateState,this);
	motionSub = nHandle.subscribe("topic_motion_cmd",10,&Joystick::enableSwitch,this);
	pub = nHandle.advertise<geometry_msgs::Twist>("cmd_vel",10);
	enable = true;
	state = STOP;
	speed = NOR_SPD;
	leftIdx1 = 0;
	leftIdx2 = 6;
	forIdx1 = 1;
	forIdx2 = 7;
	spdUpIdx = 6;
	spdDownIdx = 7;
	setSpd(0,0,0,0,0,0);
	pubRate = 100;
}
Joystick::Joystick(int axesLeftIdx,int axesForIdx,int btnSpdUpIdx,int btnSpdDownIdx){
	joySub = nHandle.subscribe("joy",10,&Joystick::updateState,this);
	motionSub = nHandle.subscribe("topic_motion_cmd",10,&Joystick::enableSwitch,this);
	pub = nHandle.advertise<geometry_msgs::Twist>("cmd_vel",10);
	enable = true;
	state = STOP;
	speed = NOR_SPD;
	leftIdx1 = axesLeftIdx;
	leftIdx2 = axesLeftIdx;
	forIdx1 = axesForIdx;
	forIdx2 = axesForIdx;
	spdUpIdx = btnSpdUpIdx;
	spdDownIdx = btnSpdDownIdx;
	setSpd(0,0,0,0,0,0);
	pubRate = 100;
}
Joystick::Joystick(int axesLeftIdx1,int axesLeftIdx2,int axesForIdx1, int axesForIdx2, int btnSpdUpIdx, int btnSpdDownIdx){
	joySub = nHandle.subscribe("joy",10,&Joystick::updateState,this);
	motionSub = nHandle.subscribe("topic_motion_cmd",10,&Joystick::enableSwitch,this);
	pub = nHandle.advertise<geometry_msgs::Twist>("cmd_vel",10);
	enable = true;
	state = STOP;
	speed = NOR_SPD;
	leftIdx1 = axesLeftIdx1;
	leftIdx2 = axesLeftIdx2;
	forIdx1 = axesForIdx1;
	forIdx2 = axesForIdx2;
	spdUpIdx = btnSpdUpIdx;
	spdDownIdx = btnSpdDownIdx;
	setSpd(0,0,0,0,0,0);
	pubRate = 100;
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
	if(msg->axes[leftIdx1]>0 || msg->axes[leftIdx2]>0){
		isLeft = true;
	}
	else if(msg->axes[leftIdx1]<0 || msg->axes[leftIdx2]<0){
		isRight = true;
	}
	//is forward or back
	if(msg->axes[forIdx1]>0 || msg->axes[forIdx2]>0){
		isForward = true;
	}
	else if(msg->axes[forIdx1]<0 || msg->axes[forIdx2]<0){
		isBack = true;
	}
	//is speed down or up
	if(msg->buttons[spdUpIdx] == 1){
		isSpdDown = true;
	}
	else if(msg->buttons[spdDownIdx] == 1){
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

void Joystick::enableSwitch(const std_msgs::String::ConstPtr& msg){
	if(msg->data == "autoMode" || msg->data == "softwareMode"){
		enable = false;
	}
	if(msg->data == "joystickMode"){
		enable = true;
	}
}

void Joystick::sendCmdVel(){
	if(!enable){
		return;
	}
	geometry_msgs::Twist msgCmdVel;	
	msgCmdVel.linear.x=msgCmdVel.linear.y=msgCmdVel.linear.z=0;
	msgCmdVel.angular.x=msgCmdVel.angular.y=msgCmdVel.angular.z=0;
	switch(state){
	case FORWARD:
		msgCmdVel.linear.x=linSpd[speed];
		break;
	case BACK:
		msgCmdVel.linear.x=-linSpd[speed];
		break;
	case LEFT:
		msgCmdVel.angular.z=angSpd[speed];
		break;
	case RIGHT:
		msgCmdVel.angular.z=-angSpd[speed];
		break;
	case FOR_LEFT:
		msgCmdVel.linear.x=linSpd[speed];
		msgCmdVel.angular.z=angSpd[speed];
		break;
	case FOR_RIGHT:
		msgCmdVel.linear.x=linSpd[speed];
		msgCmdVel.angular.z=-angSpd[speed];
		break;
	case BACK_LEFT:
		msgCmdVel.linear.x=-linSpd[speed];
		msgCmdVel.angular.z=angSpd[speed];
		break;
	case BACK_RIGHT:
		msgCmdVel.linear.x=-linSpd[speed];
		msgCmdVel.angular.z=-angSpd[speed];
		break;
	case STOP:
		msgCmdVel.linear.x=0;
		msgCmdVel.angular.z=0;
		break;
	default:
		break;
	}
	pub.publish(msgCmdVel);	
}

void Joystick::start(){
	ros::Rate loopRate(pubRate);
	while(ros::ok()){
		ros::spinOnce();
		sendCmdVel();
		loopRate.sleep();
	}
}

void Joystick::setSpd(double linS1,double linS2,double linS3,double angS1,double angS2,double angS3){
	setLinSpd(linS1,linS2,linS3);
	setAngSpd(angS1,angS2,angS3);
}

void Joystick::setLinSpd(double s1,double s2,double s3){
	double l2hSpd[3] = {s1,s2,s3};
	std::sort(l2hSpd,l2hSpd+2);
	for(int i=0;i<3;i++){
		linSpd[i] = l2hSpd[i];
	}
}

void Joystick::setAngSpd(double s1,double s2,double s3){
	double l2hSpd[3] = {s1,s2,s3};
	std::sort(l2hSpd,l2hSpd+2);
	for(int i=0;i<3;i++){
		angSpd[i] = l2hSpd[i];
	}
}

void Joystick::setPubRate(int rate){
	pubRate = rate;
}

void Joystick::setEnable(bool b){
	enable = b;
}
