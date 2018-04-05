#pragma once
//C
#include "Serial.h"
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>


using namespace std;

struct CoilData {
	__int16 x;
	__int16 y;
};

union OutputData {
	CoilData coil;
	byte buf[4];
};

class CoilDriver
{

	int x_coil, y_coil;
	OutputData outBuffer;
	Serial* arduino;
	
public:
	CoilDriver();
	CoilDriver(LPCSTR portName);
	~CoilDriver();
	void stop();
	void send(int x_coil, int y_coil);
};

