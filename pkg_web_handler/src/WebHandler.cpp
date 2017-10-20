#include "WebHandler.h"

using std::string;
using namespace tinyxml2;

WebHandler::WebHandler(const char* ip,unsigned short int port){
	goalPub = nHandle.advertise<geometry_msgs::PoseStamped>("topic_goal",10);
	pathPub = nHandle.advertise<nav_msgs::Path>("topic_global_path",10);
	motionPub = nHandle.advertise<std_msgs::String>("topic_motion_cmd",10);
	poseSub = nHandle.subscribe("topic_robot_pose",10,&WebHandler::onSubPose,this);
	initPosePub = nHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",10);
        resetPub = nHandle.advertise<std_msgs::String>("syscommand",1);
	robotPose.pose.position.x = 0;
	robotPose.pose.position.y = 0;
	robotPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	while(ros::ok() &&  !sock.connectServer(ip,port) ){
	//	connecting...
	}
std::cout << "connected" << std::endl;
	while(ros::ok()){
		ros::spinOnce();
		string cmd;
		if(sock.readline(cmd)>0){
			if(cmd == "CMD_PUB_GOAL"){
				pubGoal();		
			}
			else if(cmd == "CMD_MOTION"){
				motion();
			}
			else if(cmd == "CMD_UPDATE_MAP"){
				updateMap();
			}
			else if(cmd == "CMD_GET_POSE"){
				getPose();
			}
			else if(cmd == "CMD_PUB_PATH"){
				pubPath();
			}
			else if(cmd == "CMD_UPDATE_INIT_POSE"){
				updateInitPose();
			}
		}
		
	}
}

void WebHandler::onSubPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	robotPose = *msg;
}
void WebHandler::pubGoal(){
	char* content = new char[30];
	char split = ';';
	sock.readline(content,30);
//std::cout << content << std::endl;
	double x,y,th;
	char* tmp = strtok(content,&split);
	x = atof(tmp);
	tmp = strtok(NULL,&split);
	y = atof(tmp);
	tmp = strtok(NULL,&split);
	th = atof(tmp);
	delete[] content;
	
	geometry_msgs::PoseStamped goal;
	goal.header.frame_id = "map";
	goal.pose.position.x = x;
	goal.pose.position.y = y;
	goal.pose.orientation = tf::createQuaternionMsgFromYaw(th);  
	goalPub.publish(goal);
}

void WebHandler::motion(){
	string content;
	sock.readline(content);
//std::cout << content << std::endl;
	std_msgs::String cmd;
	cmd.data = content;
	motionPub.publish(cmd);
}

void WebHandler::updateMap(){
	char* mapPath = new char[100];
	strcpy(mapPath,"/home/");
	strcat(mapPath,getlogin());
	strcat(mapPath,"/srv_rob_conf/map.xml");
//string test;
//sock.readSock(test);
	sock.recFile(mapPath);
//std::cout << "ok" << std::endl;
}

void WebHandler::getPose(){
	string strPose;
	stringstream ss;
	ss << robotPose.pose.position.x << ";" << robotPose.pose.position.y << ";" << tf::getYaw(robotPose.pose.orientation) << ";";
	ss >> strPose;
	strPose += "\n";
//std::cout << strPose << std::endl;
	sock.writeSock(strPose);
}

void WebHandler::pubPath(){
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();

	char* content = new char[300];
	char split = ';';
	sock.readline(content,300);
//std::cout << content << std::endl;
	int count;
	double x,y,th;
	char* tmp = strtok(content,&split);
	count = atoi(tmp);
	for(int i=0;i<count*3;i+=3){
		tmp = strtok(NULL,&split);
		x = atof(tmp);
		tmp = strtok(NULL,&split);
		y = atof(tmp);
		tmp = strtok(NULL,&split);
		th = atof(tmp);
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id ="map";
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
		path.poses.push_back(pose);
	}
	pathPub.publish(path);
}

void WebHandler::updateInitPose(){
	char* content = new char[30];
	char split = ';';
	sock.readline(content,30);
std::cout << content << std::endl;
	string x,y,th;
	char* tmp = strtok(content,&split);
	x = tmp;
	tmp = strtok(NULL,&split);
	y = tmp;
	tmp = strtok(NULL,&split);
	th = tmp;
	delete[] content;
/*	
	char* xmlPath = new char[100];
        strcpy(xmlPath,"/home/");
        strcat(xmlPath, getlogin());
        strcat(xmlPath, "/srv_rob_conf/initPose.xml");

	XMLDocument doc;
        int res = doc.LoadFile(xmlPath);
        XMLElement* root = doc.RootElement();
	root->SetAttribute("x",x.c_str());
	root->SetAttribute("y",y.c_str());
	root->SetAttribute("th",th.c_str());
	doc.SaveFile(xmlPath);
*/
	geometry_msgs::PoseWithCovarianceStamped initPoseMsg;
        std_msgs::String resetMsg;
	initPoseMsg.header.frame_id = "map";
	initPoseMsg.pose.pose.position.x = atof(x.c_str());
        initPoseMsg.pose.pose.position.y = atof(y.c_str());
        initPoseMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atof(th.c_str()));

	resetMsg.data = "reset";
	initPosePub.publish(initPoseMsg);
        resetPub.publish(resetMsg);
}


