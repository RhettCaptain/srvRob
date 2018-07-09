#include "GaoJi.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_base_controller");
	BaseController* baseController = new GaoJi("/dev/ttyS4");
	baseController->start();
}
