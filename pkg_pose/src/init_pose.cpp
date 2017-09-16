#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tinyxml2.h"
#include <cstring>
#include <unistd.h>

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
	initPose.pose.pose.orientation.z = atof(root->Attribute("th"));
	doc.SaveFile(xmlPath);
	return true;
}

int main(int argc,char** argv){
	ros::init(argc,argv,"node_init_pose");
	ros::NodeHandle nHandle;
	ros::Publisher initPosePub = nHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("topic_init_pose",10);

	int keepTime = 50;
	geometry_msgs::PoseWithCovarianceStamped initPoseMsg;
	char* xmlPath = new char[100];
	strcpy(xmlPath,"/home/");
	strcat(xmlPath, getlogin());
	strcat(xmlPath, "/srv_rob_conf/initPose.xml");
	while(!loadInitPose(initPoseMsg,xmlPath)){
		ROS_INFO("LOAD FAIL");
	}
	ros::Rate wait(10);
	for(int i=0;i<keepTime;i++){
		initPosePub.publish(initPoseMsg);
		wait.sleep();
	} 
}
