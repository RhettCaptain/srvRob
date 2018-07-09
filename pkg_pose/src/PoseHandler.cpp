#include "PoseHandler.h"
#include <iostream>
using namespace std;

PoseHandler::PoseHandler(){
	slamPoseSub = nh.subscribe("slam_out_pose",10,&PoseHandler::onRecSlamPose,this);
	robotPosePub = nh.advertise<geometry_msgs::PoseStamped>("topic_robot_pose",10);
	resetPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",10);
	resetCmdPub = nh.advertise<std_msgs::String>("syscommand",1);
	motionCmdPub = nh.advertise<std_msgs::String>("topic_motion_cmd",2);	

	robotPose.header.frame_id = "empty";
	robotPose.pose.position.x = 0;
	robotPose.pose.position.y = 0;
	robotPose.pose.position.z = 0;
	robotPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	
	xErrLimit = 0.3;
	yErrLimit = 0.3;
	thErrLimit = 0.3;
}

void PoseHandler::start(){
	ros::spin();
}

void PoseHandler::onRecSlamPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
//	geometry_msgs::PoseStamped pose = *msg;
	geometry_msgs::PoseStamped lastPose = robotPose;
	robotPose = *msg;
	if(robotPose.header.frame_id == "empty"){
		return;
	}
	else{/*
		if(lastPose.header.frame_id != "empty"){
			double dx = abs(robotPose.pose.position.x-lastPose.pose.position.x);
			double dy = abs(robotPose.pose.position.y-lastPose.pose.position.y);
			double dth = abs(tf::getYaw(robotPose.pose.orientation)-tf::getYaw(lastPose.pose.orientation));
			if(dx>xErrLimit || dy>yErrLimit || dth>thErrLimit){
				std_msgs::String resetMsg;
				resetMsg.data = "reset";
				geometry_msgs::PoseWithCovarianceStamped resetPoseMsg;
				resetPoseMsg.header.frame_id = "map";
				resetPoseMsg.pose.pose = lastPose.pose;
				resetPosePub.publish(resetPoseMsg);
				resetCmdPub.publish(resetMsg);
				robotPose = lastPose;
				std_msgs::String motionMsg;
				motionMsg.data = "pause";
				motionCmdPub.publish(motionMsg);
				ros::Rate wait(1);
				wait.sleep();
				wait.sleep();
				wait.sleep();
				motionMsg.data = "go on";
				motionCmdPub.publish(motionMsg);
				return;
			}
		}*/
		cout << msg->pose.position.x << "," << msg->pose.position.y << "," << tf::getYaw(msg->pose.orientation) << endl;
	robotPosePub.publish(robotPose);
	}
}


