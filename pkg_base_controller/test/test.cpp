#include "WeiNuoController.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"bc");
	BaseController* baseController = new WeiNuoController(1,1,1);
	baseController->subCmdVel();
	baseController->start();
}
