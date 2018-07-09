#ifndef LOCALPLANNER_H
#define LOCALPLANNER_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <vector>
#include <cmath>

struct Pose{
public:
	double x,y,th;
	Pose(double px=0,double py=0,double pth=0){
		x = px;
		y = py;
		th = pth;
	}
};

class LocalPlanner{
private:
	ros::NodeHandle nHandle;
	ros::Subscriber pathSub;
	ros::Subscriber obsSub;
	ros::Subscriber cmdSub;
	ros::Subscriber poseSub;
	ros::Publisher velPub;

	bool enable;
	
	int rate;
	bool obsExist;
	bool taskFin;
	bool isPause;
	std::vector<Pose> path;
	Pose robotPose;
	int pathIdx;

	double basicLinearSpd,basicAngularSpd;
	double slowLinearSpd;
	double slowDisThreshold;	//slow down when in threshold
	double disThreshold, angThreshold;	//stop when in threshold 
	double angLimit,slowAngLimit;	//turn at situ when over angle limit
	
	int spinTimes,maxSpinTimes;	//when spin times arrive max,wait
 
public:
	LocalPlanner();
	void setRate(int pRate);
	void setSpd(double lin,double ang);
	void setThreshold(double dis,double ang);
	void setAngLimit(double ang);
	void start();

private:
	void onRecPath(const nav_msgs::Path::ConstPtr& msg);
	void onRecObs(const std_msgs::Bool::ConstPtr& msg);
	void onRecCmd(const std_msgs::String::ConstPtr& msg);
	void onRecPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pubVel();
	
	double getDis(const Pose& p1,const Pose& p2);
	double getAng(const Pose& p1,const Pose& p2);
	double getBiasAng(const double robotAng,const double goalAng);

	void printState(const char* state,double linSpd,double angSpd);
	void robustSpin(int waitRate=1);
};

#endif
