#include <Wire.h>
#include <hiduniversal.h>
#include <Adafruit_PWMServoDriver.h>
#include "Coil.h"
#include "SpaceNavigatorController.h"


USBHost usb;
bool isConnected = true;
HIDUniversal Hid(&usb);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Coil coil(&pwm);

SpaceNavigatorController SpaceNavigator(&coil);

struct CoilData {
  int16_t x;
  int16_t y;
};

union InData {
  CoilData coil;
  byte buf[4];
} inBuf;


void setup() {
	Serial.begin(115200);
  
	pinMode(2,OUTPUT);
	pinMode(3,OUTPUT);
	pinMode(4,OUTPUT);
	pinMode(5,OUTPUT);
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);
	pinMode(8,OUTPUT);
	pinMode(9,OUTPUT);
	pinMode(12,OUTPUT);  //LED ST2
	pinMode(13,OUTPUT);  //LED ST1

	digitalWrite(12,0);
	digitalWrite(13,0);
	
	pwm.begin();
	pwm.setPWMFreq(400);
  pwm.setPWM(4,0,0);
  Hid.SetReportParser(0, &SpaceNavigator);

  while(usb.getUsbTaskState() != 0x90){
    usb.Task();
    if(millis() > 1000) {
      isConnected = false;
      break;
    }  
  }
  
  if(isConnected) {
    digitalWrite(13,1);
  }
}

void loop() {
  if(isConnected) {
    usb.Task();
    if(usb.getUsbTaskState() != 0x90) {
      isConnected = false;
      digitalWrite(13,0);  
    }
  }
  
  receiveSerial();
 
  if(millis() - coil.lastUpdate() > 200) {
    coil.standby();
  }
}

void serialEventRun() {
  //keep and leave blank for speed optimization  
}

void receiveSerial() {
  if (Serial.available() < 4)
     return;

  Serial.readBytes(inBuf.buf, 4);

  coil.move_computer(inBuf.coil.x, inBuf.coil.y);
}


