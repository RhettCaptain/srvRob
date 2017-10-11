#include "Socket.h"

using std::ifstream;
using std::ofstream;
using std::cin;
using std::cout;
using std::endl;
using std::ios_base;

//implement of SocketServer
SocketServer::SocketServer(const char* type){
	if(strcmp(type,"TCP") == 0){
		serverFd = socket(AF_INET,SOCK_STREAM,0);
	}
	else if(strcmp(type,"UDP") == 0){
		serverFd = socket(AF_INET,SOCK_DGRAM,0);
	}
	else{
		throw("not support type");
	}
	clientsCount = 0;

}

SocketServer::SocketServer(int domain,int type,int protocol){
	serverFd = socket(domain,type,protocol);
	clientsCount = 0;

}

void SocketServer::setAddr(const char* fileName){
	unAddr.sun_family = AF_UNIX;
	strcpy(unAddr.sun_path,fileName);
	int len = sizeof(unAddr);
	bind(serverFd,(struct sockaddr*)&unAddr,len);
}

void SocketServer::setAddr(const char* ip,unsigned short int port){
	inAddr.sin_family = AF_INET;
	inAddr.sin_port = htons(port);
	inAddr.sin_addr.s_addr = inet_addr(ip);
	int len = sizeof(inAddr);
	bind(serverFd,(struct sockaddr*)&inAddr,len);
}

void SocketServer::listenClient(int vol){
	listen(serverFd,vol);
}
bool SocketServer::acceptClient(bool block){
	int flags = fcntl(serverFd,F_GETFL,0);
	if(block){
		fcntl(serverFd,F_SETFL, ~O_NONBLOCK&flags);
	}
	else{
		fcntl(serverFd,F_SETFL, O_NONBLOCK|flags);
	}
	
	struct sockaddr sa;
	socklen_t len=0;		//must be initialized
	int clientFd = accept(serverFd,&sa,&len);
	if(clientFd == -1){
		return false;
	}
	clientsCount++;
	clientsFd.push_back(clientFd);
	clientsAddr.push_back(sa);
	clientsAddrLen.push_back(len);
	return true;
}

int SocketServer::getClientsCount(){
	return clientsCount;
}

struct sockaddr SocketServer::getClientAddr(int idx){
	if(idx>=0 && idx< clientsCount){
		return clientsAddr[idx];
	}
	else{
		throw("no such client!");
	}
}
void SocketServer::clearClient(int idx){
	if(idx<0){
		clientsCount = 0;
		clientsFd.clear();
		clientsAddr.clear();
		clientsAddrLen.clear();
	}
	else{
		if(idx>=0 && idx< clientsCount){
			std::vector<int>::iterator itFd = clientsFd.begin()+idx;
			clientsFd.erase(itFd);
			std::vector<struct sockaddr>::iterator itAddr = clientsAddr.begin()+idx;
			clientsAddr.erase(itAddr);
			std::vector<socklen_t>::iterator itLen = clientsAddrLen.begin()+idx;
			clientsAddrLen.erase(itLen);
			clientsCount--;
		}
		else{
			std::cout << "invalid idx!" << std::endl;
		}
	}
}

void SocketServer::setBlock(bool on,int clientIdx){
	int flags = fcntl(clientsFd[clientIdx],F_GETFL,0);
	if(on){
		flags &= ~O_NONBLOCK;
	}
	else{
		flags |= O_NONBLOCK;
	}
	fcntl(clientsFd[clientIdx],F_SETFL,flags);
}

int SocketServer::readSock(char* buffer,int len, int clientIdx){
	int sock = clientsFd[clientIdx];
	int nBytes = read(sock,buffer,len);
	return nBytes;
}

int SocketServer::readSock(std::string &buffer,int len,int clientIdx){
	char* cBuffer = new char[len];
	int nBytes = readSock(cBuffer,len,clientIdx);
	if(nBytes == -1){
		delete[] cBuffer;
		return -1;
	}
	else{
		buffer.insert(0,cBuffer,nBytes);
		delete[] cBuffer;
		return nBytes;
	}
}

int SocketServer::readline(char* buffer, int len, char eol,int clientIdx){
	char ch=' ';
	int idx=0;
	while(ros::ok() && ch!=eol){
		if(readSock(&ch,1,clientIdx)>0){
			if(ch!=eol){
				buffer[idx++] = ch;
			}
		}
		if(idx==len){
			std::cout << "no eol in len bytes" << std::endl;
			break;
		}
	}
	return idx;
}

int SocketServer::readline(std::string &buffer,int len,char eol,int clientIdx){
	char* cBuffer = new char[len];
	int nBytes = readline(cBuffer,len,eol,clientIdx);
	if(nBytes == -1){
		delete[] cBuffer;
		return -1;
	}
	else{
		buffer.insert(0,cBuffer,nBytes);
		delete[] cBuffer;
		return nBytes;
	}
}

int SocketServer::recFile(const char* dstPath,int clientIdx){
	std::string fileContext;
	ofstream fout;
	fout.open(dstPath,ios_base::out|ios_base::trunc);
	readline(fileContext,10000,EOF,clientIdx);
//	fileContext.erase(fileContext.end()-1);
	fout << fileContext;
	fout.close();
	return fileContext.length();
}

int SocketServer::writeSock(const char* buffer,int len,int clientIdx){
	int sock = clientsFd[clientIdx];
	int nBytes = write(sock,buffer,len);
	return nBytes;
}

int SocketServer::writeSock(const std::string &buffer, int clientIdx){
	int sock = clientsFd[clientIdx];
	for(int i=0;i<buffer.size();i++){
		write(sock,&buffer[i],1);
	}
	return buffer.size();
}

int SocketServer::sendFile(const char* srcPath, int clientIdx){
	ifstream fin;
	fin.open(srcPath);
	std::string fileContext;
	getline(fin,fileContext,(char)EOF);
	fileContext += EOF;
	writeSock(fileContext,clientIdx);
	fin.close(); 
	return fileContext.length()-1;
}
bool SocketServer::closeServer(int how){
	if( shutdown(serverFd,how) == -1){
		return false;
	}
	return true;
}

bool SocketServer::closeClient(int idx,int how){
	if(idx < 0){
		std::vector<int>::iterator itFd;
		for(itFd = clientsFd.begin();itFd != clientsFd.end(); itFd++){
			if( shutdown(*itFd,how) == -1){
				return false;
			}
		}
	}
	else if(idx>=0 && idx< clientsCount){
        	std::vector<int>::iterator itFd = clientsFd.begin()+idx;
		if( shutdown(*itFd,how) == -1){
			return false;
		}
	}
	else{
		std::cout << "invalid idx" << std::endl;
		return false;
	}
	return true;
}

//implement of SocetClient

SocketClient::SocketClient(const char* type){
	if(strcmp(type,"TCP") == 0){
		clientFd = socket(AF_INET,SOCK_STREAM,0);
	}
	else if(strcmp(type,"UDP") == 0){
		clientFd = socket(AF_INET,SOCK_DGRAM,0);
	}
	else{
		throw("not support type");
	}
}


SocketClient::SocketClient(int domain,int type,int protocol){
	clientFd = socket(domain,type,protocol);
}

bool SocketClient::connectServer(const char* fileName){
	serverUnAddr.sun_family = AF_UNIX;
	strcpy(serverUnAddr.sun_path,fileName);
	int len = sizeof(serverUnAddr);
	if( connect(clientFd,(struct sockaddr*)&serverUnAddr,len) == -1 ){
		return false;
	}
	return true;
}

bool SocketClient::connectServer(const char* ip,unsigned short int port){
	serverInAddr.sin_family = AF_INET;
	serverInAddr.sin_port = htons(port);
	serverInAddr.sin_addr.s_addr = inet_addr(ip);
	int len = sizeof(serverInAddr);
	if( connect(clientFd,(struct sockaddr*)&serverInAddr,len) == -1 ){
		return false;
	}
	return true;
}





void SocketClient::setBlock(bool on){
	int flags = fcntl(clientFd,F_GETFL,0);
	if(on){
		flags &= ~O_NONBLOCK;
	}
	else{
		flags |= O_NONBLOCK;
	}
	fcntl(clientFd,F_SETFL,flags);
}
	
int SocketClient::readSock(char* buffer, int len){
	int nBytes = read(clientFd,buffer,len);
	return nBytes;
}

int SocketClient::readSock(std::string &buffer, int len){
	char* cBuffer = new char[len];
	int nBytes = readSock(cBuffer,len);
	if(nBytes == -1){
		delete[] cBuffer;
		return -1;
	}
	else{
		buffer.insert(0,cBuffer,nBytes);
		delete[] cBuffer;
		return nBytes;
	}
}

int SocketClient::readline(char* buffer,int len,char eol){
	setBlock(false);
	char ch=' ';
	int idx=0;
	while(ros::ok() && ch!=eol){
		if(readSock(&ch,1)>0){
			if(ch!=eol){
				buffer[idx++] = ch;
			}
		}
		if(idx==len){
			std::cout << "no eol in len bytes" << std::endl;
			break;
		}
	}
	return idx;
}

int SocketClient::readline(std::string &buffer,int len,char eol){
	char* cBuffer = new char[len];
	int nBytes = readline(cBuffer,len,eol);
	if(nBytes == -1){
		delete[] cBuffer;
		return -1;
	}
	else{
		buffer.insert(0,cBuffer,nBytes);
		delete[] cBuffer;
		return nBytes;
	}
}

int SocketClient::recFile(const char* dstPath){
	std::string fileContext;
	ofstream fout;
	fout.open(dstPath,ios_base::out|ios_base::trunc);
	readline(fileContext,10000,EOF);
//	fileContext.erase(fileContext.end()-1);
	fout << fileContext;
	fout.close();
	return fileContext.length();
}
int SocketClient::writeSock(const char* buffer,int len){
	int nBytes = write(clientFd,buffer,len);
	return nBytes;
}

int SocketClient::writeSock(const std::string &buffer){
	for(int i=0;i<buffer.size();i++){
		write(clientFd,&buffer[i],1);
	}
	return buffer.size();
}

int SocketClient::sendFile(const char* srcPath){
	std::string fileContext;
	ifstream fin;
	fin.open(srcPath);
	getline(fin,fileContext,(char)EOF);
	fileContext += EOF;
	writeSock(fileContext);
	return fileContext.length()-1;
}
bool SocketClient::close(int how){
	if( shutdown(clientFd,how) == -1){
		return false;
	}
	return true;
}

