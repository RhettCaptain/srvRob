#include "../Socket.h"
#include <iostream>
#include <unistd.h>
using namespace std;
int main(){
	SocketServer ss;
	ss.setAddr("127.0.0.1",9999);
	ss.listenClient();
	while(!ss.acceptClient()){};
	cout << "accepted" << endl;
/*	while(1){
		string send,get;
		send = "i'm server,send 2 client\n";
		ss.writeSock(send);
		ss.readline(get);
		cout << "get: " << get << endl;
		sleep(1);
	}*/
	ss.recFile("serverFile");
	ss.sendFile("serverFile");
}
