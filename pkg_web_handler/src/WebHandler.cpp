#include "WebHandler.h"

using std::string;

WebHandler::WebHandler(const char* ip,unsigned short int port){
	goalPub = nHandle.advertise<geometry_msgs::PoseStamped>("topic_goal",10);
	while(ros::ok() &&  !sock.connectServer(ip,port) ){
	//	connecting...
	}
std::cout << "connected" << std::endl;
	while(ros::ok()){
		string cmd;
		if(sock.readline(cmd)>0){
			if(cmd == "CMD_PUB_GOAL"){
				pubGoal();		
			}
			else if(cmd == "CMD_MOTION"){
				motion();
			}
		}
		
	}
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
