#ifndef SOCKET_H
#define SOCKET_H

#include <sys/types.h>
#include <sys/socket.h> 
#include <sys/un.h>
#include <netinet/in.h>
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <ros/ros.h>

using namespace std;
class SocketServer{
private:
	int serverFd;
	struct sockaddr_un unAddr;
	struct sockaddr_in inAddr;

	std::vector<int> clientsFd;
	int clientsCount;
	std::vector<struct sockaddr> clientsAddr;
	std::vector<socklen_t> clientsAddrLen;

public:
	SocketServer(const char* type = "TCP");
	SocketServer(int domain,int type,int protocol=0);
	void setAddr(const char* fileName);
	void setAddr(const char* ip,unsigned short int port);
	void listenClient(int vol=5);
	bool acceptClient(bool block = true);
	int getClientsCount();
	struct sockaddr getClientAddr(int idx);
	void clearClient(int idx=-1);
	
	void setBlock(bool on,int clientIdx=0);

	int readSock(char* buffer, int len,int clientIdx=0);
	int readSock(std::string &buffer, int len=100,int clientIdx=0);
	int readline(char* buffer,int len,char eol='\n',int clientIdx=0);
	int readline(std::string &buffer,int len=100,char eol='\n',int clientIdx=0);
	int recFile(const char* dstPath,int clientIdx=0);
	
	int writeSock(const char* buffer,int len,int clientIdx=0);
	int writeSock(const std::string &buffer,int clientIdx = 0);
	int sendFile(const char* srcPath,int clientIdx=0);
	
	bool closeServer(int how=SHUT_RDWR);
	bool closeClient(int idx=-1,int how=SHUT_RDWR);
	
private:
};

class SocketClient{
private:
	int clientFd;

	struct sockaddr_un serverUnAddr;
	struct sockaddr_in serverInAddr;
public:
	SocketClient(const char* type ="TCP");
	SocketClient(int domain,int type,int protocol=0);
	bool connectServer(const char* fileName);
	bool connectServer(const char* ip,unsigned short int port);
	
	void setBlock(bool on);
	
	int readSock(char* buffer, int len);
	int readSock(std::string &buffer, int len=100);
	int readline(char* buffer,int len,char eol='\n');
	int readline(std::string &buffer,int len=100,char eol='\n');
	int recFile(const char* dstPath);

	int writeSock(const char* buffer,int len);
	int writeSock(const std::string &buffer);
	int sendFile(const char* srcPath);

	bool close(int how=SHUT_RDWR);
private:
};


#endif
