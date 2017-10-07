#include "WebHandler.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_web_handler");
	WebHandler webHandler("192.168.1.250",9999);
	
}
