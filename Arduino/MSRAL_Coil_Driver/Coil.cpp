#include "Coil.h"

Coil::Coil(Adafruit_PWMServoDriver *pwmDriver) : pwm(pwmDriver) 
{
  Kp_translation = 8; // default gain
  Kp_rotation = 0.01;
  angle = 0; // default angle, pointing right
  offset_up = offset_left = offset_down = offset_right = 0;
}

void Coil::translation(int pwr_x, int pwr_y) {
}

void Coil::rotation(int delt_angle) {
}

void Coil::move_controller(int x, int y, int delt_angle) {
  int up = 0, left = 0, down = 0, right = 0;

  if(abs(delt_angle) > 60) {
    /**********************rotation*******************start*/
    angle += delt_angle * Kp_rotation;
    angle %= 360;
          
    int rotation_x = cos(angle * PI / 180) * PWM_ROTATION; 
    left = -rotation_x;
    right = rotation_x;
    
    int rotation_y = sin(angle * PI / 180) * PWM_ROTATION; 
    up = rotation_y;
    down = -rotation_y;
    /**********************rotation*********************end*/
  } else {  
    /*********************translation*****************start*/
    if(abs(x) > abs(y)) {
      if(x < 0){
        left = (-x + offset_left) * Kp_translation;
        if(-x > 50) 
          angle = 180;
      } else{
        right = (x + offset_right) * Kp_translation;
        if(x > 50) 
          angle = 0;
      }
    }else {  
      if(y < 0){
        up = (-y + offset_up) * Kp_translation;
        if(-y > 50)
          angle = 90;
      } else{
        down = (y + offset_down) * Kp_translation;
        if(y > 50)
          angle = 270;
      }
    }
    /*********************translation******************end*/       
  }
  Serial.println(angle);
  setPWM(up, left, down, right);  
}

void Coil::move_computer(int x, int y) {
  int up = 0, left = 0, down = 0, right = 0;

  if(x > 0)
    right = x;
  else 
    left = -x;

  if(y > 0)
    down = y;
  else
    up = -y;
  setPWM(up, left, down, right); 
}

void Coil::setPWM(int up, int left, int down, int right) {
    digitalWrite(2, up > 0);
    pwm->setPWM(0,0, abs(up));
    
    digitalWrite(3, left > 0);
    pwm->setPWM(1,0, abs(left));
    
    digitalWrite(4, down > 0);
    pwm->setPWM(2,0,abs(down));
    
    digitalWrite(5, right > 0);
    pwm->setPWM(3,0, abs(right));

    lastTime = millis();
}

unsigned long Coil::lastUpdate() {
  return lastTime;
}

void Coil::setOffset(int up, int left, int down, int right) {
  offset_up = up;
  offset_left = left;
  offset_down = down;
  offset_right = right;
}

void Coil::standby() {
  setPWM(0, 0, 0, 0);  
}

void Coil::setGain(double gain) {
  Kp_translation = gain;  
}
