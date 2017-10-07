#include "WebHandler.h"

using std::string;

WebHandler::WebHandler(const char* ip,unsigned short int port){
	goalPub = nHandle.advertise<geometry_msgs::PoseStamped>("topic_goal",10);
	while(ros::ok() &&  !sock.connectServer(ip,port) ){
	//	connecting...
	}
	while(ros::ok()){
		string cmd;
		if(sock.readSock(cmd)>0){
			if(cmd == "CMD_PUB_GOAL"){
				pubGoal();		
			}
			else if(cmd == "CMD_
		}
		
	}
}

void WebHandler::pubGoal(){
	std::cout << "test" << std::endl;
}
