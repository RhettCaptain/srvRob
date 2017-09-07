
#include "SerialPort.h"
#include <vector>
#include <cstdio>
#include <string>
using namespace std;
int main(){
	SerialPort* sp = new SerialPort("/dev/ttyUSB0");
	sp->openPort();
	vector<unsigned char> vec(7);
	vec[0] = 0x1b;
	vec[1] = 0x20;
	vec[2] = 0x01;//0x07;
	vec[3] = 0x00;//0x57;
	vec[4] = 0x9a;//0x0a;
	vec[5] = 0x71;//0x90;
	vec[6] = 0x05;//0xa8;
	sp->setBaud(115200);
//	sp->setBlock(false);
//	sp->setInMode('r');
//	sp->setBlock(true);
//	sp->setOutMode('r');
//	vector<unsigned char> vec2(14);
//	char* vec2 = new char[14];
	string vec2;
	for(int i=0;i<1000000;i++){
		sp->writePort(vec);
//		usleep(50*1000);
		int n = sp->readPort(vec2,14);
	//	sp->flush();
	if(n>0){
		cout << "get: " << n << endl;
		for(int i=0;i<14;i++){
		//	cout<<(unsigned int)vec2[i] << " ";
			printf("%x ",vec2[i]); 
		}
		cout << endl;
	}
	}
}

