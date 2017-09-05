#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <iostream>

class Joystick{
private:
	ros::NodeHandle nHandle;
	ros::Subscriber sub;
	enum State{FORWARD,BACK,LEFT,RIGHT,FOR_LEFT,FOR_RIGHT,BACK_LEFT,BACK_RIGHT,STOP} state;
	enum Speed{HIGH_SPD,NOR_SPD,LOW_SPD} speed;
	void updateState(const sensor_msgs::Joy::ConstPtr& msg);
	void sendCmdVel();
public:
	Joystick();
	void start();
	


};
