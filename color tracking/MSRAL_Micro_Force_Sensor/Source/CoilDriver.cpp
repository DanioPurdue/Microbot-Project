#include "CoilDriver.h"

CoilDriver::CoilDriver()
{
}

CoilDriver::CoilDriver(LPCSTR portName)
{
	arduino = new Serial(portName);  
	if (arduino->IsConnected())
		cout << "Arduino connected." << endl;
	else
		cout << "Failed to connect Arduino." << endl;
}

CoilDriver::~CoilDriver()
{
}

void CoilDriver::stop()
{
	send(0,0);
}

void CoilDriver::send(int x_coil, int y_coil)
{
	outBuffer.coil.x = x_coil;
	outBuffer.coil.y = y_coil;
	arduino->WriteData(outBuffer.buf, 4);
}
