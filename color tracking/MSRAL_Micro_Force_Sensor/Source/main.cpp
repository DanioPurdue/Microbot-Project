#define COILPOWER 1800
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <string.h>
#include "Serial.h"
#include "CoilDriver.h"
//opencv
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;


int main(int argc, char* argv[])
{
	
	CoilDriver coilDriver("COM3");    // adjust as needed
	namedWindow("CV");
	while (true) {
		switch (waitKey(0)) {
		case 122: //down left
			
			coilDriver.send(-800, 800);
			cout << "down left" << endl;
			break;
		case 120: //down
			coilDriver.send(0, COILPOWER);
			cout << "down" << endl;
			break;
		case 99: //down right
			coilDriver.send(800, 800);
			cout << "down right" << endl;
			break;
		case 97: //left
			coilDriver.send(-COILPOWER, 0);
			cout << "left" << endl;
			break;
		case 100: //right
			coilDriver.send(COILPOWER, 0);
			cout << "right" << endl;
			break;
		case 113: //up left
			coilDriver.send(-800, -800);
			cout << "up left" << endl;
			break;
		case 119: //up
			coilDriver.send(0, -COILPOWER);
			cout << "up "<< endl;
			break;
		case 101: //up right
			coilDriver.send(800, -800);
			cout << "up right" << endl;
			break;
		}
	}

		return 0;
}

