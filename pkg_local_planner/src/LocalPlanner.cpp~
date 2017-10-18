#include "LocalPlanner.h"

LocalPlanner::LocalPlanner(){
	pathSub = nHandle.subscribe("topic_global_path",10,&LocalPlanner::onRecPath,this);
	obsSub = nHandle.subscribe("topic_obstacle",10,&LocalPlanner::onRecObs,this);
	cmdSub = nHandle.subscribe("topic_motion_cmd",10,&LocalPlanner::onRecCmd,this);
	poseSub = nHandle.subscribe("topic_robot_pose",10,&LocalPlanner::onRecPose,this);
	velPub = nHandle.advertise<geometry_msgs::Twist>("cmd_vel",10);

	rate = 20;
	obsExist = false;
	taskFin = true;
	isPause = false;
	pathIdx = 0;	

	basicLinearSpd = 0.25;
	basicAngularSpd = 0.1;
	disThreshold = 0.05;
	angThreshold = 0.15;
	angLimit = 0.5;
}

void LocalPlanner::setRate(int pRate){
	rate = pRate;
}

void LocalPlanner::setSpd(double lin,double ang){
	basicLinearSpd = lin;
	basicAngularSpd = ang;
}

void LocalPlanner::setThreshold(double dis,double ang){
	disThreshold = dis;
	angThreshold = ang;
}

void LocalPlanner::setAngLimit(double ang){
	angLimit = ang;
}

void LocalPlanner::start(){
	pubVel();
}

void LocalPlanner::onRecPath(const nav_msgs::Path::ConstPtr& msg){
	taskFin = false;
	path.clear();
	int pCount = msg->poses.size();
	for(int i=0;i<pCount;i++){
		double x = msg->poses[i].pose.position.x;
		double y = msg->poses[i].pose.position.y;
		double th = tf::getYaw(msg->poses[i].pose.orientation);
		Pose* tmp = new Pose(x,y,th);
		path.push_back(*tmp);
	}
} 

void LocalPlanner::onRecObs(const std_msgs::Bool::ConstPtr& msg){
	obsExist = msg->data;
}

void LocalPlanner::onRecCmd(const std_msgs::String::ConstPtr& msg){
	if(msg->data == "pause"){
		isPause = true;
	}
	else if(msg->data == "go on"){
		isPause = false;
	}
	else if(msg->data == "stop"){
		taskFin = true;
		path.clear();
		pathIdx = 0;
	}
}

void LocalPlanner::onRecPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	robotPose.x = msg->pose.position.x;
	robotPose.y = msg->pose.position.y;
	robotPose.th = tf::getYaw(msg->pose.orientation);
}

void LocalPlanner::pubVel(){
	ros::Rate wait(rate);
	while(ros::ok()){
		ros::spinOnce();
		
		geometry_msgs::Twist vel;
		if(isPause || taskFin){
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.angular.z = 0;
			velPub.publish(vel);
			wait.sleep();
//std::cout << "Pause or fin" << std::endl;
		}
		else if(path.size() > 0 && getDis(robotPose,path[pathIdx]) <= disThreshold){
//std::cout << "in goal" << std::endl;
			//next goal and task finish rule
			if(pathIdx == path.size()-1){
				//task finish
				double biasAng = path[pathIdx].th - robotPose.th;
				int spinDir = biasAng / fabs(biasAng);
				while(ros::ok() && fabs(biasAng) > angThreshold){
					ros::spinOnce();
					biasAng = path[pathIdx].th - robotPose.th;
					spinDir = biasAng / fabs(biasAng);
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = spinDir * basicAngularSpd;
					velPub.publish(vel);
					wait.sleep();
std::cout << biasAng << "," << robotPose.th << std::endl;
				}
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.angular.z = 0;
				velPub.publish(vel);
				wait.sleep();
				path.clear();
				pathIdx = 0;
				taskFin = true;
			}
			else{
				//next goal
				pathIdx++;
			}
			
		}
		else if(obsExist){
//std::cout << "obstacle" << std::endl;
			//avoid obstacle strategy
			while(ros::ok() && obsExist){
				ros::spinOnce();
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.angular.z = basicAngularSpd;
				velPub.publish(vel);
				wait.sleep();
			}
			int keepTimes = 20;
			for(int i=0;i<keepTimes;i++){
				ros::spinOnce();
				if(obsExist){
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = basicAngularSpd;
					velPub.publish(vel);
					break;
				}
				vel.linear.x = basicLinearSpd;
				vel.linear.y = 0;
				vel.angular.z = 0;
				velPub.publish(vel);
				wait.sleep();
			}
		}
		else{
			//control strategy	
			double biasAng = getAng(robotPose,path[pathIdx]) - robotPose.th;
			int spinDir = biasAng / fabs(biasAng);
			if(fabs(biasAng) > angLimit){
				while(ros::ok() && fabs(biasAng) > angThreshold){
					ros::spinOnce();
					biasAng = getAng(robotPose,path[pathIdx]) - robotPose.th;
					spinDir = biasAng / fabs(biasAng);
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = spinDir * basicAngularSpd;
					velPub.publish(vel);
//std::cout << "adjust" << std::endl;
					wait.sleep();
				}
			}
			else{
				vel.linear.x = basicLinearSpd;
				vel.linear.y = 0;
				vel.angular.z = spinDir * basicAngularSpd;
				velPub.publish(vel);
				wait.sleep();
//std::cout <<"normal" << std::endl;
			}
		}

	}
}

double LocalPlanner::getDis(const Pose& p1,const Pose& p2){
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	double dis = sqrt(dx*dx + dy*dy);
	return dis;
}

double LocalPlanner::getAng(const Pose& p1,const Pose& p2){
	double angle;
	if(p1.x == p2.x){
		if(p2.y >= p1.y){
			angle = M_PI/2;
		}
		else{
			angle = -M_PI/2;
		}
	}
	else{
		angle = atan2(p2.y-p1.y,p2.x-p1.x);
	}
	return angle;
}

