#include "AJ.h"
#include <string>
#include "std_msgs/Bool.h"

int dis;

void onRecAJ(const std_msgs::Int16::ConstPtr& msg){
	dis = msg->data;	
}

int main(int argc,char** argv){
	ros::init(argc,argv,"node_obstacle_sensor");
	ros::NodeHandle nHandle;
	ros::Subscriber sub = nHandle.subscribe("topic_aj1",100,onRecAJ);
	dis=65535;
	ros::Publisher pub = nHandle.advertise<std_msgs::Bool>("topic_obstacle",10);

	int obsThr = 500;	//mm
	
	while(ros::ok()){
		ros::spinOnce();
		std_msgs::Bool obsExist;
		obsExist.data = false;
		if(dis < obsThr){
			obsExist.data = true;	
		} 
		pub.publish(obsExist);
	}
}

/* thread may have problem
void* createAJ(void* idx){
	std::string suffix = (char*)idx;
	std::string addr="/dev/ttyUSB"+suffix;
	std::string topic="aj"+suffix;
	AJ* aj=new AJ(addr.c_str(),topic.c_str());
	aj->start();
}

int dis[3];

void onRecAJ0(const std_msgs::Int16::ConstPtr& msg){
	dis[0] = msg->data;	
}
void onRecAJ1(const std_msgs::Int16::ConstPtr& msg){
	dis[1] = msg->data;	
}
void onRecAJ2(const std_msgs::Int16::ConstPtr& msg){
	dis[2] = msg->data;	
}

int main(int argc,char** argv){
	ros::init(argc,argv,"node_obstacle_sensor");
	ros::NodeHandle nHandle;
	ros::Subscriber sub0 = nHandle.subscribe("aj0",100,onRecAJ0);
	dis[0]=65535;
	dis[1]=65535;
	dis[2]=65535;
	ros::Subscriber sub1 = nHandle.subscribe("aj1",100,onRecAJ1);
	ros::Subscriber sub2 = nHandle.subscribe("aj2",100,onRecAJ2);
	ros::Publisher pub = nHandle.advertise<std_msgs::Bool>("topic_obstacle",10);

	pthread_t tid;
	int err;
	err = pthread_create(&tid,NULL,createAJ,(void*)"0");
	err = pthread_create(&tid,NULL,createAJ,(void*)"1");
	err = pthread_create(&tid,NULL,createAJ,(void*)"2");

	int obsThr = 700;	//mm
	
	while(ros::ok()){
		ros::spinOnce();
		std_msgs::Bool obsExist;
		obsExist.data = false;
		for(int i=0;i<3;i++){
			if(dis[i] < obsThr){
				obsExist.data = true;	
			}
		} 
		pub.publish(obsExist);
	}
	
	
}
*/
