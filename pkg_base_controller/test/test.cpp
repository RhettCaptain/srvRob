#include "WeiNuoController.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"bc");
	BaseController* baseController = new WeiNuoController();
	baseController->subCmdVel();
	baseController->start();
}
