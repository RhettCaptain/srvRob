#ifndef EAICONTROLLER_H
#define EAICONTROLLER_H

#include "BaseController.h"

class EAIController:public BaseController{
private:
	
public:
	EAIController();
	void onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
	void start();
};


#endif
