
#include "SerialPort.h"
#include <vector>
#include <cstdio>
#include <string>
using namespace std;
int main(){
	SerialPort* sp = new SerialPort("/dev/ttyUSB0");
	sp->openPort();
//	vector<unsigned char> vecFL(9);
	char vecFL[9];
	vecFL[0] = 0x1b;
	vecFL[1] = 0x01;
	vecFL[2] = 0x07;//0x07;
	vecFL[3] = 0x57;//0x57;
	vecFL[4] = 0x0a;//0x0a;
	vecFL[5] = 0x90;
	vecFL[6] = 0xa8;
	vecFL[7] = 0xaf;
	vecFL[8] = 0x05;
	sp->setBaud(115200);

	vector<unsigned char> vecF(9);
	vecF[0] = 0x1b;
	vecF[1] = 0x10;
	vecF[2] = 0x04;
	vecF[3] = 0xb0;
	vecF[4] = 0x04;
	vecF[5] = 0xb0;
	vecF[6] = 0x60;
	vecF[7] = 0xe6;
	vecF[8] = 0x05;
//	sp->setBlock(false);
//	sp->setInMode('r');
//	sp->setBlock(true);
//	sp->setOutMode('r');
//	vector<unsigned char> vec2(14);
//	char* vec2 = new char[14];
//	string vec2;
//	for(int i=0;i<1000000;i++){
		sp->writePort(vecFL);
//		usleep(50*1000);
//		int n = sp->readPort(vec2,14);
	//	sp->flush();
/*	if(n>0){
		cout << "get: " << n << endl;
		for(int i=0;i<14;i++){
		//	cout<<(unsigned int)vec2[i] << " ";
			printf("%x ",vec2[i]); 
		}
		cout << endl;
	}
	}*/
}

