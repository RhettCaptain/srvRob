#ifndef GAOJI_H
#define GAOJI_H

#include "BaseController.h"
#include "SerialPort.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

class GaoJi:public BaseController{
private:
	SerialPort* motor;
	ros::NodeHandle nHandle;
	ros::Publisher pub;
	ros::Time pubTime;
	
	enum MotorDir{FF=0x10,FB=0x11,BF=0x00,BB=0x01}motorDir;
	void crc16Modbus(uchar *p, int len,uchar* hCrc,uchar* lCrc);
	void openMotor(const char* motorPort,int baud=9600,int dataBits=8,int stopBits=1,char parity='n');
	void closeMotor();
	void move(int leftRotateSpd,int rightRotateSpd);
	void pubFakeOdometerMsg(const geometry_msgs::Twist::ConstPtr& msg);
	void leftMove(int rotateSpd);
	void rightMove(int rotateSpd);
	void setLeftBackTime(int t);
	void setRightBackTime(int t);
	void leftBack(bool on);
	void rightBack(bool on);
	void enable(bool on);
public:
	GaoJi(const char* motorPort,double pWheelRadius=0.060,double pWheelDis=0.4,double pGearRatio=25);
	GaoJi(const GaoJi& wn);
	GaoJi& operator=(const GaoJi& wn);
	void onRecCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
	void start();

	
};


#endif
