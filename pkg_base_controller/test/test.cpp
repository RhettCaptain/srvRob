#include "WeiNuoController.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"bc");
	BaseController* baseController = new WeiNuoController("/dev/ttyUSB1");
	baseController->start();
}
