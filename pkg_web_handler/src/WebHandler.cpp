#include "WebHandler.h"

using std::string;

WebHandler::WebHandler(const char* ip,unsigned short int port){
	goalPub = nHandle.advertise<geometry_msgs::PoseStamped>("topic_goal",10);
	while(ros::ok() &&  !sock.connectServer(ip,port) ){
	//	connecting...
	}
std::cout << "debug10" << std::endl;	
	while(ros::ok()){
		string cmd;
		if(sock.readSock(cmd)>0){
std::cout << cmd << std::endl;
			if(cmd == "CMD_PUB_GOAL"){
				pubGoal();		
			}
		}
		
	}
}

void WebHandler::pubGoal(){
	std::cout << "test" << std::endl;
}
