#include "WebHandler.h"

using std::string;

WebHandler::WebHandler(const char* ip,unsigned short int port){
	goalPub = nHandle.advertise<geometry_msgs::PoseStamped>("topic_goal",10);
	poseSub = nHandle.subscribe("topic_robot_pose",10,&WebHandler::onSubPose,this);
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
std::cout << "get pose " << std::endl;
				getPose();
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
std::cout << content << std::endl;
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
std::cout << content << std::endl;
}

void WebHandler::updateMap(){
	char* mapPath = new char[100];
	strcpy(mapPath,"/home/");
	strcat(mapPath,getlogin());
	strcat(mapPath,"/srv_rob_conf/map.xml");
//string test;
//sock.readSock(test);
	sock.recFile(mapPath);
std::cout << "ok" << std::endl;
}

void WebHandler::getPose(){
	string strPose;
	stringstream ss;
	ss << robotPose.pose.position.x << ";" << robotPose.pose.position.y << ";" << tf::getYaw(robotPose.pose.orientation) << ";";
	ss >> strPose;
	strPose += "\n";
std::cout << strPose << std::endl;
	sock.writeSock(strPose);
}
