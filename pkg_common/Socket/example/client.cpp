#include "../Socket.h"
#include <iostream>
#include <unistd.h>
using namespace std;
int main(){
	SocketClient sc;
	while(!sc.connectServer("127.0.0.1",9999)){
		cout << "not connect" << endl;
	}
	cout << "connected" << endl;
/*	while(1){
		string send,get;
		send = "i'm client,send 2 server\n";
		sc.writeSock(send);
		sc.readline(get);
		cout << "get: " << get << endl;
		sleep(1);
	}*/
	sc.sendFile("clientFile");
	cout << "send over" << endl;
	sc.recFile("clientFile");
	sc.close();
}
