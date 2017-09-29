#include "SerialPort.h"


SerialPort::SerialPort(const char* pPortName)
{
	fd = -1;
	portName = pPortName;
	openState = false;
	blockState = false;
	outTime = 5;
	baud = 115200;
	dataBits = 8;
	stopBits = 1;
	parity = 'n';		
	inMode = 'c';
	outMode = 'c';
}

SerialPort::~SerialPort()
{
	
}


int SerialPort::openPort()
{
	fd = open(portName,O_RDWR|O_NOCTTY|O_NDELAY);
	if(-1 == fd)
	{
		cout << "open" << portName <<"fail\n";
		return -1;
	}
	openState = true;
}

int SerialPort::openPort(int flag)
{
	fd = open(portName,flag);
	
	if(-1 == fd)
	{
		cout << "open" << portName << "fail\n";
		return -1;
	}
	openState = true;
}

int SerialPort::openPort(int flag,int perms)
{
	fd = open(portName,flag,perms);
	
	if(-1 == fd)
	{
		cout << "open" << portName << "fail\n";
		return -1;
	}
	openState = true;
	
}

void SerialPort::closePort()
{
	if(fd != -1 && openState == true)
	{
		close(fd);	
	}
	else 
	{
		cout << "no port is open!\n";	
	}	
	openState = false;
}

void SerialPort::setBaud(int pBaud)
{
	//save old config
	if( tcgetattr(fd,&options) != 0)
	{
		cout << "please open port first!\n";
		return;
	}
	
	int tmp;
	switch(pBaud)
	{
	case 4800:
		tmp = B4800;
		break; 
	case 9600:
		tmp = B9600;
		break;
	case 19200:
		tmp = B19200;
		break;
	case 38400:
		tmp = B38400;
		break;
	case 57600:
		tmp = B57600;
		break;
	case 115200:
		tmp = B115200;
		break;
	default:
		tmp = -1;
		cout << "unsupported baud\n";
		return;
	}
	baud = pBaud;
	cfsetispeed(&options, tmp);
	cfsetospeed(&options, tmp);
	options.c_cflag |= (CLOCAL | CREAD);
	tcflush(fd,TCIFLUSH);	//flush
	//set immediately
	if(tcsetattr(fd,TCSANOW,&options) !=0)
	{
		cout << "set baud fail\n";
		return;
	}
}

int SerialPort::getBaud()
{
	return baud;
}

void SerialPort::setDataBits(int pDataBits)
{
	//save old options
	if( tcgetattr(fd,&options) != 0 )
	{
		cout << "please open port first\n";
		return;
	}
	//set databits
	options.c_cflag &= ~CSIZE;	//clear
	switch(pDataBits)
	{
	case 5:
		options.c_cflag |= CS5;
		break;
	case 6:
		options.c_cflag |= CS6;
		break;
	case 7:
		options.c_cflag |= CS7;
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	defalut:
		cout<< "unsupported data size\n";
		return;
	}
	dataBits = pDataBits;	
	//set immediately
	tcflush(fd,TCIFLUSH);	//flush
	if(tcsetattr(fd,TCSANOW,&options) != 0)
	{
		cout << "set databits fail\n";
		return;
	}
}

int SerialPort::getDataBits()
{
	return dataBits;
}

void SerialPort::setStopBits(int pStopBits)
{
	//save old options
	if( tcgetattr(fd,&options) != 0)
	{
		cout  << "please open port first\n";
		return;
	}
	//set stopbits
	switch(pStopBits)
	{
	case 1:
		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		cout << "unsupported stopBits\n";
		return;
	}
	stopBits = pStopBits;
	//set immediately
	tcflush(fd,TCIFLUSH);
	if(tcsetattr(fd,TCSANOW,&options) != 0)
	{
		cout << "set stopbits fail\n";
		return;
	}
}

int SerialPort::getStopBits()
{
	return stopBits;
}

void SerialPort::setParity(char pParity)
{
	//save old options
	if( tcgetattr(fd,&options) != 0)
	{
		cout << "please open port first\n";
		return;
	}
	//set parity
	switch(pParity)
	{
	case 'n':	//none
	case 'N':
		options.c_cflag &= ~PARENB;
		options.c_iflag &= ~INPCK;
		break;
	case 'o':	//odd 
	case 'O':
		options.c_cflag |= (PARODD | PARENB);
		options.c_iflag |= INPCK;
		break;
	case 'e':	//even
	case 'E':
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_iflag |= INPCK;
		break;
	case 's':	//space
	case 'S':
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		cout << "unsupported parity\n";
		return;
	}
	parity = pParity;
	//set immediately
	tcflush(fd, TCIFLUSH);
	if( tcsetattr(fd,TCSANOW,&options) != 0)
	{
		cout << "set parity fail\n";
		return;
	}
}

char SerialPort::getParity()
{
	return parity;
}

void SerialPort::setPort()
{
	setBaud(baud);
	setDataBits(dataBits);
	setStopBits(stopBits);
	setParity(parity);
}

void SerialPort::setPort(int pBaud,int pDataBits,int pStopBits,char pParity)
{
	setBaud(pBaud);
	setDataBits(pDataBits);
	setStopBits(pStopBits);
	setParity(pParity);
}

bool SerialPort::isOpen()
{
	return openState;
}
	
bool SerialPort::isBlock()
{
	return blockState;
}

void SerialPort::setBlock(bool on)
{
	int flags;
	flags = fcntl(fd,F_GETFL,0);
	if(on)
	{
		flags &= ~O_NONBLOCK;
	}
	else
	{
		flags |= O_NONBLOCK;
	}
	blockState = on;
	fcntl(fd,F_SETFL,flags);
}

void SerialPort::setOutTime(float pOutTime)
{
	//save old options
	if( tcgetattr(fd,&options) != 0)
	{
		cout << "please open port first\n";
		return;
	}
	//setOutTime
	options.c_cc[VTIME] = (int)(pOutTime*10);
	outTime = options.c_cc[VTIME]/10;
	//set immediately
	if(tcsetattr(fd, TCSANOW, &options) != 0)
	{
		cout << "set out time fail\n";
		return;
	}
}

float SerialPort::getOutTime()
{
	return outTime;
}

void SerialPort::setInMode(char mode)
{
	//save old options
	if( tcgetattr(fd,&options) != 0)
	{
		cout << "please open port first\n";
		return;
	}
	switch(mode)
	{
	case 'c':	//classic
	case 'C':
		options.c_lflag |= (ICANON | ECHO | ECHOE);
		inMode = 'c';
		break;
	case 'r':	//raw
	case 'R':
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		inMode = 'r';
		break;
	default:
		cout << "only support c/C for classic mode and r/R for raw mode\n";
		return;
	}
	//set immediately
	if(tcsetattr(fd,TCSANOW,&options)!=0)
	{
		cout << "set input mode fail\n";
		return;
	}
}

char SerialPort::getInMode()
{
	return inMode;
}

void SerialPort::setOutMode(char mode)
{
	//save old options
	if( tcgetattr(fd,&options) != 0)
	{
		cout << "please open port first\n";
		return;
	}
	switch(mode)
	{
	case 'c':	//classic
	case 'C':
		options.c_oflag |= OPOST;
		outMode = 'c';
		break;
	case 'r':	//raw
	case 'R':
		options.c_oflag &= ~OPOST;
		outMode = 'r';
		break;
	default:	
		cout << "only support c/C for classic mode and r/R for raw mode\n";
		return;
	}
	//set immediately
	if(tcsetattr(fd,TCSANOW,&options)!=0)
	{
		cout << "set output mode fail\n";
		return;
	}
}

char SerialPort::getOutMode()
{	
	return outMode;
}

int SerialPort::readPort(char* buffer, int len)
{
	if(fd == -1)
	{
		cout << "please open port first\n";
		return -1;
	}
	int nBytes = read(fd,buffer,len);
	return nBytes;
}

int SerialPort::readPort(string &buffer,int len){
	char* cBuffer = new char[len];
	int nBytes = readPort(cBuffer,len);
	if(nBytes == -1){
		return -1;
	}
	else{
		buffer.insert(0,cBuffer,nBytes);
		return nBytes;
	}
}

string SerialPort::readPort2str(int len){
	string buffer;
	readPort(buffer,len);
	return buffer;
}

int SerialPort::readPort(vector<uchar> &buffer,int len){
	char* cBuffer = new char[len];
	buffer.resize(len);
	int nBytes = readPort(cBuffer,len);
	if(nBytes == -1){
		return -1;
	}
	else{
		for(int i=0;i<nBytes;i++){
			buffer[i] = (uchar)cBuffer[i];
		}
		return nBytes;
	}
}

int SerialPort::readline(char* buffer,int len,char eol){
	char ch=' ';
	int idx=0;
	while(ch!=eol){
		if(readPort(&ch,1)>0){
			if(ch!=eol){
				buffer[idx++] = ch;		
			}
		}
		if(idx==len){
			cout << "no eol in len bytes" << endl;
			break;
		}
	}
	return idx;
}

int SerialPort::readline(string &buffer,int len,char eol){
	char* cBuffer = new char[len];
	int nBytes = readline(cBuffer,len,eol);
	if(nBytes == -1){
		return -1;
	}
	else{
		buffer.insert(0,cBuffer,nBytes);
		delete[] cBuffer;
		return nBytes;
	}
}

string SerialPort::readline(int len,char eol){
	string buffer;
	readline(buffer,len,eol);
	return buffer;
}

int SerialPort::writePort(const char* buffer, int len){
	if(fd == -1){
		cout << "please open port first\n";
		return -1;
	}
	int nBytes = write(fd,buffer,len);
	return nBytes;
}

int SerialPort::writePort(const string &buffer){
	if(fd == -1){
		cout << "please open port first\n";
		return -1;
	}
	for(int i=0;i<buffer.size();i++){
		write(fd,&buffer[i],1);
	}
	return buffer.size();
}

int SerialPort::writePort(const vector<uchar> &buffer){
	if(fd == -1){
		cout << "please open port first\n";
		return -1;
	}
	for(int i=0;i<buffer.size();i++){
		write(fd,&buffer[i],1);
	}
	return buffer.size();
}

void SerialPort::flush()
{
	tcflush(fd,TCIOFLUSH);
}

void SerialPort::inFlush()
{
	tcflush(fd,TCIFLUSH);
}

void SerialPort::outFlush()
{
	tcflush(fd,TCOFLUSH);
}
