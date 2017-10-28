#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "unistd.h"

int main(int argc,char** argv){
	ros::init(argc,argv,"node_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",2);
	for(int i=0;i<5000/20;i++){
		geometry_msgs::Twist vel;
		vel.linear.x = 5;
		pub.publish(vel);
		usleep(20*1000);
	}
}
