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
	int x_coil, y_coil;
	int angle;
	int maxForceCounter;
	float maxForce;
	CoilDriver *coilDriver;
	Point lastPos;
	enum States { reach, push, rotate, halt} state, preState;
	void do_reach();
	void do_push();
	void do_rotate();
	double norm(Point2d pt);
	Point2d uvector(Point2d pt);
	double Control::getAngle(Point pt);
	Point target;
	Point objectWayPoint;
	Point objectPos;
	double kp_pwm,kd_pwm,ki_pwm;
	string msg;


public:

	void setTarget(Point pushPoint, Point dirPoint);
	Control(CoilDriver*);
	~Control();
	void update(MicroObject);//control.update(new MicroObject(center, radius, angle, force))
	void start();
	void stop();

};

