#include "AJ.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_aj");
	AJ* aj = new AJ("/dev/usb_aj","topic_aj1");
	aj->start();
}
