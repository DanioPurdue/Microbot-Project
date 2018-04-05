#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//C
#include <stdio.h>
#include <math.h>
//C++
#include <iostream>
#include <sstream>
#include <vector>
#include "CoilDriver.h"
#include "MicroObject.h"

using namespace std;
using cv::Mat;
using cv::Point;
using cv::Point2d;

class Control
{
	MicroObject robot;
	int x_coil, y_coil, pwm;
	int maxForceCounter;
	int maxForce;
	CoilDriver *coilDriver;
	Point2d last_error, error_sum;
	enum States { state_move, state_push, state_halt} state;
	void move();
	void push();
	void translate(Point target);
	void rotate(int angle);
	double norm(Point2d pt);
	Point2d uvector(Point2d pt);
	void resetPID();
	double Control::getAngle(Point pt);
	int sign(int number);
	Point objectWayPoint;
	Point objectPos;
	double Kp,Kd,Ki;
	string msg;


public:

	void setTarget(Point pushPoint, Point dirPoint);
	Control(CoilDriver*);
	~Control();
	void update(MicroObject);//control.update(new MicroObject(center, radius, angle, force))
	void start();
	void stop();

};

