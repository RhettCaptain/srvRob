#include "LocalPlanner.h"
 
LocalPlanner::LocalPlanner(){
	pathSub = nHandle.subscribe("topic_global_path",10,&LocalPlanner::onRecPath,this);
	obsSub = nHandle.subscribe("topic_obstacle",10,&LocalPlanner::onRecObs,this);
	cmdSub = nHandle.subscribe("topic_motion_cmd",10,&LocalPlanner::onRecCmd,this);
	poseSub = nHandle.subscribe("topic_robot_pose",10,&LocalPlanner::onRecPose,this);
	velPub = nHandle.advertise<geometry_msgs::Twist>("cmd_vel",10);

	enable = true;

	rate = 20;
	obsExist = false;
	taskFin = true;
	isPause = false;
	pathIdx = 0;	

	basicLinearSpd = 0.25;//0.25;
	basicAngularSpd = 0.5;//0.1;
	slowLinearSpd = 0.1;//0.1;
	slowDisThreshold = 0.15;	//slow down when dis less than this
	disThreshold = 0.05;		//arrive when dis less than this
	angThreshold = 0.2;		//arrive when ang less than this
	angLimit = 0.5;			//fix dir when ang bigger than this
	slowAngLimit = 0.3;		

	spinTimes = 0;
	maxSpinTimes = 20;
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
	//return path
	for(int i=0;i<pCount-1;i++){
		double x = msg->poses[pCount-i-2].pose.position.x;
		double y = msg->poses[pCount-i-2].pose.position.y;
		double th = tf::getYaw(msg->poses[pCount-i-2].pose.orientation);
		Pose* tmp = new Pose(x,y,th);
		path.push_back(*tmp);
	}
printf("///////////////%d/////////////////////\n",pCount);
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
std::cout << "########################################" << std::endl;
		path.clear();
		pathIdx = 0;
	}
	else if(msg->data == "joystickMode" || msg->data == "softwareMode"){
		enable = false;
	}
	else if(msg->data == "autoMode"){
		enable = true;
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
		
		double tempLinSpd,tempAngSpd,tempAngLimit;
		if(path.size() > 0 && getDis(robotPose,path[pathIdx]) <= slowDisThreshold){
			tempLinSpd = slowLinearSpd;
			tempAngSpd = basicAngularSpd;
			tempAngLimit = slowAngLimit;
		}
		else{
			tempLinSpd = basicLinearSpd;
			tempAngSpd = basicAngularSpd;
			tempAngLimit = angLimit;
		}
		geometry_msgs::Twist vel;
		if(!enable){
			wait.sleep();
			continue;
		}
		else if(isPause || taskFin){
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.angular.z = 0;
			velPub.publish(vel);
			wait.sleep();
printState("Pause or fin",0,0);
		}
		else if(path.size() > 0 && getDis(robotPose,path[pathIdx]) <= disThreshold){
			//next goal and task finish rule
			if(pathIdx == path.size()/2){
				//task finish
				double biasAng = getBiasAng(robotPose.th,path[pathIdx].th);
				int spinDir = biasAng / fabs(biasAng);
				while(ros::ok() && fabs(biasAng) > angThreshold){
					ros::spinOnce();
					if(isPause){
						spinTimes = 0;
						vel.linear.x = 0;
						vel.linear.y = 0;
						vel.angular.z = 0;
						velPub.publish(vel);
						wait.sleep();
						continue;
					}
					if(taskFin){
						break;
					}
					biasAng = getBiasAng(robotPose.th,path[pathIdx].th);
					spinDir = biasAng / fabs(biasAng);
					robustSpin();
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = spinDir * tempAngSpd;
					velPub.publish(vel);
					wait.sleep();
printState("in the last goal dis and adjusting ang",0,vel.angular.z);
				}
				spinTimes = 0;
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.angular.z = 0;
				velPub.publish(vel);
				wait.sleep();
				pathIdx++;
				usleep(5000*1000);
			//	path.clear();
			//	pathIdx = 0;
			//	taskFin = true;
printState("arrive the last second goal,waiting for taking meals",0,0);
			}else if(pathIdx == path.size()-1){
				path.clear();
				pathIdx = 0;
				taskFin = true;
printState("arrive the last goal",0,0);
			}
			else{
				//next goal
				pathIdx++;
printState("arrive a temp goal",0,0);
			}
			
		}
		else if(obsExist){
			//avoid obstacle strategy
			while(ros::ok() && obsExist){
				ros::spinOnce();
				if(isPause){
					spinTimes = 0;
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = 0;
					velPub.publish(vel);
					wait.sleep();
					continue;
				}
				if(taskFin){
					break;
				}
				robustSpin();
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.angular.z = tempAngSpd;
				velPub.publish(vel);
				wait.sleep();
printState("meet obstacle and adjusting",0,vel.angular.z);
			}
			int keepTimes = 200;
			for(int i=0;i<keepTimes;i++){
				ros::spinOnce();
				if(isPause){
					spinTimes = 0;
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = 0;
					velPub.publish(vel);
					wait.sleep();
					continue;
				}
				if(taskFin){
					break;
				}
				if(obsExist){
					robustSpin();
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = tempAngSpd;
					velPub.publish(vel);
					break;
				}
				spinTimes = 0;
				vel.linear.x = tempLinSpd;
				vel.linear.y = 0;
				vel.angular.z = 0;
				velPub.publish(vel);
				wait.sleep();
printState("remove obstacle",tempLinSpd,0);
			}
		}
		else{
			//control strategy	
			double biasAng;
			if(getDis(robotPose,path[pathIdx])>slowDisThreshold){
				biasAng = getBiasAng(robotPose.th,getAng(robotPose,path[pathIdx]));
			}else{
			//	biasAng = getBiasAng(robotPose.th,path[pathIdx].th);
				biasAng = 0.01;
			}
			
			int spinDir = biasAng / fabs(biasAng);
			if(fabs(biasAng) > angLimit){
				while(ros::ok() && fabs(biasAng) > angThreshold){
					ros::spinOnce();
					if(isPause){
						spinTimes = 0;
						vel.linear.x = 0;
						vel.linear.y = 0;
						vel.angular.z = 0;
						velPub.publish(vel);
						wait.sleep();
						continue;
					}
					if(taskFin){
						break;
					}
					if(getDis(robotPose,path[pathIdx])>slowDisThreshold){
						biasAng = getBiasAng(robotPose.th,getAng(robotPose,path[pathIdx]));
					}else{
				//		biasAng = getBiasAng(robotPose.th,path[pathIdx].th);
						biasAng = 0.01;
					}
					spinDir = biasAng / fabs(biasAng);
				//	spinDir = 1;
					robustSpin();
					vel.linear.x = 0;
					vel.linear.y = 0;
					vel.angular.z = spinDir * tempAngSpd;
					velPub.publish(vel);
					wait.sleep();
printState("normal area adjusting ang",0,vel.angular.z);
				}
			}
			else{
				spinTimes = 0;
				vel.linear.x = tempLinSpd;
				vel.linear.y = 0;
				vel.angular.z = spinDir *  tempAngSpd * 0.05;
				velPub.publish(vel);
				wait.sleep();
printState("normal area moving",tempLinSpd,vel.angular.z);
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

double LocalPlanner::getBiasAng(const double robotAng,const double goalAng){
	double bias = goalAng - robotAng;
	if(bias > M_PI){
		bias -= 2*M_PI;
	}
	else if(bias <= -M_PI){
		bias += 2*M_PI;
	}
	return bias;
}

void LocalPlanner::printState(const char* state,double linSpd,double angSpd){
	std::cout << "--------------" << std::endl;
	std::cout << "state: " << state << std::endl;
	if(path.size()==0){
		std::cout << "no path received" << std::endl;
		std::cout << "xxxxxxxxxxxxxxxx" << std::endl;
		return;	
	}
	std::cout << "dis to goal: " << getDis(robotPose,path[pathIdx]) << std::endl;
	std::cout << "biasAng to goal: " << getBiasAng(robotPose.th,path[pathIdx].th) << std::endl;
	std::cout << "biasAng to linkLine: "<< getBiasAng(robotPose.th,getAng(robotPose,path[pathIdx])) << std::endl;
	std::cout << "linSpd: " << linSpd << ";angSpd: " << angSpd << std::endl;
	std::cout << "xxxxxxxxxxxxxxxx" << std::endl;
}

void LocalPlanner::robustSpin(int waitRate){
	ros::Rate wait(waitRate);
	spinTimes++;
	if(spinTimes >= maxSpinTimes){
		wait.sleep();
		spinTimes = 0;
	}
}
