#include "Control.h"
#define MAXPOWER 2000
#define MINPOWER 400
#define PI 3.14159265

void Control::setTarget(Point pushPoint, Point dirPoint)
{
	objectPos = pushPoint;
	objectWayPoint = dirPoint;
	start();
}

double Control::norm(Point2d pt)
{
	return sqrt(pt.x*pt.x + pt.y*pt.y);
}

double Control::getAngle(Point pt) {
	return ((int)(atan2(-pt.y, pt.x) * 180 / PI + 360)) % 360;
}

Point2d Control::uvector(Point2d pt)
{
	pt *= 100;
	double dist = norm(pt);
	return Point2d(pt.x / dist, pt.y / dist);
}

Control::Control(CoilDriver *coilDriver)
{
	this->coilDriver = coilDriver;
}

void Control::start()
{
		state = reach;
		maxForce = 25;
		maxForceCounter = 0;
		kp_pwm = 1;
		kd_pwm = 0.1;
		lastPos = cv::Point(0,0);
}


void Control::stop() {
	preState = state;
	state = halt;
	coilDriver->stop();
}

void Control::update(MicroObject robot)
{
	robot.radius *= 1.5;
	this->robot = robot;

	x_coil = 600 * kp_pwm, y_coil = 600 * kp_pwm;

	cout << "robot position " << robot.center << " object position: " << objectPos <<endl;
	switch (state) {
		case reach: do_reach(); cout << "state: reach" << endl; break;
		case push: do_push(); cout << "state: push" << endl; break;
		case rotate: do_rotate();return;
		case halt: cout << "state: halt " << msg << endl; return;
	}

	Point2d dist = target - robot.center;
	x_coil = kp_pwm * dist.x / sqrt(dist.x*dist.x + dist.y*dist.y) + kd_pwm * (robot.center - lastPos).x;
	//x_coil = dist.x / sqrt(dist.x*dist.x + dist.y*dist.y);
	y_coil = kp_pwm * dist.y / sqrt(dist.x*dist.x + dist.y*dist.y) + kd_pwm * (robot.center - lastPos).y;
	//y_coil = dist.y / sqrt(dist.x*dist.x + dist.y*dist.y);
	cout << "Coil PWM:" << x_coil << " " << y_coil << endl;
	coilDriver->send(x_coil, y_coil);
	lastPos = robot.center;
}

void Control::do_reach()
{
	if(robot.isInside(objectPos)) {
		state = push;
		return;
	}

	angle = getAngle(objectPos - robot.center);
	if (abs(robot.angle - angle) > 90) {
		preState = state;
		state = rotate;
		return;
	}

	//move robot to object
	target = objectPos;

	if (abs(objectPos.x - objectWayPoint.x) > abs(objectPos.y - objectWayPoint.y)) {
		if (abs(robot.center.y - target.y) < 2) {
			target.y = robot.center.y;
		}	else {
			target.x = robot.center.x;
		}
	}	else {
		if (abs(robot.center.x - target.x) < 2) {
			target.x = robot.center.x;
		}	else {
			target.y = robot.center.y;
		}
	}

	//this works as ki_pwm
	if (norm(robot.center - lastPos) < 15) {
		kp_pwm += 0.1;
		cout << "kp_pwm increased to " << kp_pwm << endl;
	}
	else {
		//kp_pwm = 1;
		//cout << "kp_pwm decreased to " << kp_pwm << endl;

		kp_pwm -= 0.1; // reset kp_pwm
		kp_pwm = (kp_pwm > 1 ? kp_pwm : 1);
		cout << "kp_pwm decreased to " << kp_pwm << endl;
	}

}

void Control::do_push()
{
	Point2d alignment = uvector(objectPos - objectWayPoint) + uvector(objectPos - robot.center);

	// check if robot and push point are aligned
	if (norm(alignment) < 0.1) {
		target = objectWayPoint;
		angle = getAngle(objectWayPoint - objectPos);
		if (abs(robot.angle - angle) > 60) {
			preState = state;
			state = rotate;
			return;
		}
	}/*
	else {
		preState = state;
		state = reach;
		return;
	}*/

	if (norm(robot.center - target) < 20) {
		preState = state;
		state = halt;
		msg = "Mission complete.";
	}


	if (robot.force >= maxForce) {
		maxForceCounter++;
	}

	if (maxForceCounter > 0) {
		preState = state;
		state = halt;
		maxForce = robot.force;
		msg = "Maximum force applied.";
	}
	kp_pwm += 0.02;
}


void Control::do_rotate()
{
	if (abs(robot.angle - angle) < 100) {
		state = preState;
		preState = rotate;
		return;
	}

	int nextAngle = robot.angle + 90;
	nextAngle %= 360;

	cout << "Rotate from " << robot.angle << " to " << nextAngle << " desired angle is " << angle << endl;
	x_coil = cos(nextAngle* PI / 180.0) * MINPOWER;
	y_coil = -sin(nextAngle * PI / 180.0) * MINPOWER;
	coilDriver->send(x_coil, y_coil);
	cout << "Coil PWM:" << x_coil << " " << y_coil << endl;
}


Control::~Control()
{
}
