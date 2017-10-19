#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <algorithm>

class Joystick{
private:
	ros::NodeHandle nHandle;
	ros::Subscriber joySub;
	ros::Subscriber motionSub;
	ros::Publisher pub;
	enum State{FORWARD,BACK,LEFT,RIGHT,FOR_LEFT,FOR_RIGHT,BACK_LEFT,BACK_RIGHT,STOP} state;
	enum Speed{LOW_SPD,NOR_SPD,HIGH_SPD} speed;

	bool enable;
	int leftIdx1,leftIdx2;
	int forIdx1,forIdx2;
	int spdUpIdx,spdDownIdx;

	double linSpd[3];	//low 2 high
	double angSpd[3];	//low 2 high

	int pubRate;

	void updateState(const sensor_msgs::Joy::ConstPtr& msg);
	void enableSwitch(const std_msgs::String::ConstPtr& msg);
	void sendCmdVel();
public:
	Joystick();
	Joystick(int axesLeftIdx,int axesForIdx,int btnSpdUpIdx,int btnSpdDownIdx);
	Joystick(int axesLeftIdx1,int axesLeftIdx2,int axesForIdx1, int axesForIdx2, int btnSpdUpIdx, int btnSpdDownIdx);

	void start();
	
	void setSpd(double linS1,double linS2,double linS3,double angS1,double angS2,double angS3);
	void setLinSpd(double s1,double s2, double s3);
	void setAngSpd(double s1,double s2, double s3);

	void setPubRate(int rate);
	void setEnable(bool b);
};
