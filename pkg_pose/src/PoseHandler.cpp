#include "PoseHandler.h"
#include <iostream>
using namespace std;

PoseHandler::PoseHandler(){
	slamPoseSub = nh.subscribe("slam_out_pose",10,&PoseHandler::onRecSlamPose,this);
	robotPosePub = nh.advertise<geometry_msgs::PoseStamped>("topic_robot_pose",10);
	robotPose.header.frame_id = "empty";
	robotPose.pose.position.x = 0;
	robotPose.pose.position.y = 0;
	robotPose.pose.position.z = 0;
	robotPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
}

void PoseHandler::start(){
	ros::spin();
}

void PoseHandler::onRecSlamPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
//	geometry_msgs::PoseStamped pose = *msg;
	robotPose = *msg;
	if(robotPose.header.frame_id == "empty"){
		return;
	}
	else{
		cout << msg->pose.position.x << "," << msg->pose.position.y << "," << tf::getYaw(msg->pose.orientation) << endl;
	robotPosePub.publish(robotPose);
	}
}


