#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <vector>

typedef unsigned char uchar;
using std::vector;
using std::string;
using std::cout;
using std::endl;

class SerialPort
{
private: 
	int fd;		
	const char* portName;	
	bool openState;
	bool blockState;
	float outTime;
	int baud;
	int dataBits;
	int stopBits;
	char parity;
	char inMode;
	char outMode;
	struct termios options;
public:
	SerialPort(const char* pPortName);
	~SerialPort();
	
	int openPort();
	int openPort(int flag);
	int openPort(int flag,int perms);
	void closePort();
	
	void setBaud(int pBaud);
	int getBaud();
	void setDataBits(int pDataBits);
	int getDataBits();
	void setStopBits(int pStopBits);
	int getStopBits();
	void setParity(char pParity);
	char getParity();
	void setPort();
	void setPort(int pBaud,int pDataBits,int pStopBits,char pParity);
	
	bool isOpen();
	bool isBlock();
	void setBlock(bool on);
	void setOutTime(float pOutTime);
	float getOutTime();

	//@mode:'c'-classic mode
	//     'r'-raw mode
	void setInMode(char mode);
	char getInMode();
	void setOutMode(char mode);
	char getOutMode();

	int readPort(char* buffer,int len);
	int readPort(string &buffer,int len=100);
	string readPort2str(int len=100);
	int readPort(vector<uchar> &buffer,int len=100);
	int readline(char* buffer,int len,char eol='\n');
	int readline(string &buffer,int len=100,char eol='\n');
	string readline(int len=100,char eol='\n');
	
	int writePort(const char* buffer,int len);
	int writePort(const string &buffer);
	int writePort(const vector<uchar> &buffer);
	

	void flush();
	void inFlush();
	void outFlush();
};


#endif
