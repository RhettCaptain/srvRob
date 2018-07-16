#include "Odometry.h"
#include "tinyxml2.h"
using namespace tinyxml2;

bool loadInitPose(double* x,double* y,double* th,char* xmlPath){
        XMLDocument doc;
        int res = doc.LoadFile(xmlPath);
        if(res!=0){
                return false;
        }
        XMLElement* root = doc.RootElement();
        *x = atof(root->Attribute("x"));
        *y = atof(root->Attribute("y"));
       	*th = atof(root->Attribute("th"));
        doc.SaveFile(xmlPath);
        return true;
}

int main(int argc,char** argv){
	ros::init(argc,argv,"node_odometry");
	//loadl initpose
	char* xmlPath = new char[100];
        strcpy(xmlPath,"/home/");
      //  strcat(xmlPath, getlogin());
        strcat(xmlPath, "canfu");
        strcat(xmlPath, "/srv_rob_conf/initPose.xml");
	double x,y,th;
        while(ros::ok() && !loadInitPose(&x,&y,&th,xmlPath)){
                ROS_INFO("LOAD FAIL");
        }
	
	Odometry odometry(x,y,th);;
	odometry.start(50);
}
