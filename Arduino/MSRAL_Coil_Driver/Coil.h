#if !defined(__MSRAL_COIL_H__)
#define __MSRAL_COIL_H__
#define PWM_ROTATION 600

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class Coil {
    Adafruit_PWMServoDriver *pwm;
    
public:
    Coil(Adafruit_PWMServoDriver *pwm);
    void translation(int pwr_x, int pwr_y);
    void rotation(int delt_angle);
    void move_controller(int x, int y, int delt_angle);
    void move_computer(int x, int y);
    void setPWM(int up, int left, int down, int right);
    void setOffset(int up, int left, int down, int right);
    void setGain(double gain);
    void standby();
    unsigned long lastUpdate();
    
    double Kp_rotation, Kp_translation;
    int angle;
    int offset_up, offset_left, offset_down, offset_right;
    unsigned long lastTime;
};

#endif // __MSRAL_COIL_H__
