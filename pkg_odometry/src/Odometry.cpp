#include "Odometry.h"

Odometry::Odometry(double x,double y,double th){
	odometerSub = nHandle.subscribe("topic_odometer_sensor",10,&Odometry::onRecOdometerMsg,this);
	odometryPub = nHandle.advertise<nav_msgs::Odometry>("odom",10);
	
	odometryPose.position.x = x;
	odometryPose.position.y = y;
	odometryPose.position.z = 0;
	odometryPose.orientation.x = 1;
	odometryPose.orientation.y = 0;
	odometryPose.orientation.z = 0;
	odometryPose.orientation.w = 0;
	
	odometryTwist.linear.x = 0;
	odometryTwist.linear.y = 0;
	odometryTwist.linear.z = 0;
	odometryTwist.angular.x = 0;
	odometryTwist.angular.y = 0;
	odometryTwist.angular.z = 0;
}

Odometry::Odometry(const geometry_msgs::Pose& initPose){
	odometerSub = nHandle.subscribe("topic_odometer_sensor",10,&Odometry::onRecOdometerMsg,this);
	odometryPub = nHandle.advertise<nav_msgs::Odometry>("odom",10);
	odometryPose = initPose;
	
	odometryTwist.linear.x = 0;
	odometryTwist.linear.y = 0;
	odometryTwist.linear.z = 0;
	odometryTwist.angular.x = 0;
	odometryTwist.angular.y = 0;
	odometryTwist.angular.z = 0;
}

void Odometry::start(int spinRate){
	ros::Rate loop(spinRate);
	while(ros::ok()){
		ros::spinOnce();
		pubOdom();
		broadcastTf();
		loop.sleep();
	}
}
void Odometry::pubOdom(){
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	odom.pose.pose = odometryPose;
	odom.twist.twist = odometryTwist;
	
	odometryPub.publish(odom);
}

void Odometry::broadcastTf(){
	geometry_msgs::TransformStamped base2OdomTf;
	base2OdomTf.header.stamp = ros::Time::now();
	base2OdomTf.header.frame_id = "odom";
	base2OdomTf.child_frame_id = "base_link";
	base2OdomTf.transform.translation.x = odometryPose.position.x;
	base2OdomTf.transform.translation.y = odometryPose.position.y;
	base2OdomTf.transform.translation.z = odometryPose.position.z;
	base2OdomTf.transform.rotation  = tf::createQuaternionMsgFromYaw(tf::getYaw(odometryPose.orientation));
/*
std::cout << base2OdomTf.transform.translation.x << std::endl;	
std::cout << base2OdomTf.transform.translation.y << std::endl;	
std::cout << base2OdomTf.transform.translation.z << std::endl;	
std::cout << base2OdomTf.transform.rotation.x << std::endl;	
std::cout << base2OdomTf.transform.rotation.y << std::endl;	
std::cout << base2OdomTf.transform.rotation.z << std::endl;	
std::cout << base2OdomTf.transform.rotation.w << std::endl;	
std::cout << "-------------------------" << std::endl;	
*/
	tfBroadcaster.sendTransform(base2OdomTf);
	base2OdomTf.header.stamp = ros::Time::now();
	base2OdomTf.header.frame_id = "base_link";
	base2OdomTf.child_frame_id = "laser_frame";
	base2OdomTf.transform.translation.x = 0;
	base2OdomTf.transform.translation.y = 0;
	base2OdomTf.transform.translation.z = 0;
	base2OdomTf.transform.rotation  = tf::createQuaternionMsgFromYaw(0);
	tfBroadcaster.sendTransform(base2OdomTf);
}

void Odometry::onRecOdometerMsg(const nav_msgs::Odometry::ConstPtr& msg){
	double deltaDis = msg->pose.pose.position.x;
	double deltaAng = msg->pose.pose.orientation.z;
	double rawAng = tf::getYaw(odometryPose.orientation);
	odometryPose.position.x += deltaDis * cos(deltaAng / 2 + rawAng);
	odometryPose.position.y += deltaDis * sin(deltaAng / 2 + rawAng);
	double newAng = rawAng + deltaAng;
	normalizeAng(newAng);
	odometryPose.orientation = tf::createQuaternionMsgFromYaw(newAng);
	odometryTwist = msg->twist.twist;
}

void Odometry::normalizeAng(double& ang){
	while(ang < -M_PI){
		ang += 2 * M_PI;
	}
	while(ang > M_PI){
		ang -= 2 * M_PI;
	}
}
