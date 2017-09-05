#include "BaseController.h"

BaseController::BaseController(){
	topic_cmd_vel = "cmd_vel";
}

void BaseController::setTopicCmdVel(string topic){
	topic_cmd_vel = topic;
}

void BaseController::setTopicCmdVel(char* topic){
	topic_cmd_vel = topic;
}

virtual void start(){
	cmdVelSub.subscribe(topic_cmd_vel,10,move);
}
