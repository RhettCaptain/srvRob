#include "WeiNuoController.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"bc");
	BaseController* baseController = new WeiNuoController("/dev/ttyS4");
	baseController->start();
}
