#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"
#include "tinyxml2.h"
#include <cstring>
#include <unistd.h>
#include "tf/tf.h"

using namespace tinyxml2;

bool loadInitPose(geometry_msgs::PoseWithCovarianceStamped& initPose,char* xmlPath){
	XMLDocument doc;
	int res = doc.LoadFile(xmlPath);
	if(res!=0){
		return false;		
	}
	XMLElement* root = doc.RootElement();
	initPose.pose.pose.position.x = atof(root->Attribute("x"));
	initPose.pose.pose.position.y = atof(root->Attribute("y"));
	initPose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atof(root->Attribute("th")));
	doc.SaveFile(xmlPath);
	return true;
}

int main(int argc,char** argv){
	ros::init(argc,argv,"node_init_pose");
	ros::NodeHandle nHandle;
	ros::Publisher initPosePub = nHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",10);
	ros::Publisher resetPub = nHandle.advertise<std_msgs::String>("syscommand",1);
	geometry_msgs::PoseWithCovarianceStamped initPoseMsg;
	std_msgs::String resetMsg;
	initPoseMsg.header.frame_id = "map";
	char* xmlPath = new char[100];
	strcpy(xmlPath,"/home/");
	strcat(xmlPath, getlogin());
	strcat(xmlPath, "/srv_rob_conf/initPose.xml");
	while(ros::ok() && !loadInitPose(initPoseMsg,xmlPath)){
		ROS_INFO("LOAD FAIL");
	}
	resetMsg.data = "reset";
	int delayTime = 3;
	ros::Rate wait(1);
	for(int i=0;i<delayTime;i++){
		wait.sleep();
	} 
	initPosePub.publish(initPoseMsg);
	resetPub.publish(resetMsg);
}
