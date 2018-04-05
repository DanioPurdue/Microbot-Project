#include "Control.h"
#define MAXPOWER 2000
#define MINPOWER 500
#define MAXFORCE 40
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
	//return 0 ~ 360
	return ((int)(atan2(-pt.y, pt.x) * 180 / PI + 360)) % 360;
}

int Control::sign(int number) {
	if (number == 0)
		return 0;
	else
		return number>0 ? 1:-1;
}

Point2d Control::uvector(Point2d pt)
{
	pt *= 100;
	double dist = norm(pt);
	return Point2d(pt.x / dist, pt.y / dist);
}

void Control::resetPID() {
	Kp = 7;
	Kd = 20;
	Ki = 0.3;

	last_error = cv::Point(0, 0);
	error_sum = last_error;
}

Control::Control(CoilDriver *coilDriver)
{
	this->coilDriver = coilDriver;
}

void Control::start()
{
		state = state_move;
		maxForce = MAXFORCE;
		maxForceCounter = 0;
		resetPID();
}


void Control::stop() {
	state = state_halt;
	coilDriver->stop();
}

void Control::update(MicroObject robot)
{
	this->robot = robot;
	
	switch (state) {
		case state_move: cout << "state: move" << endl; move(); break;
		case state_push: cout << "state: push" << endl; push(); break;
		case state_halt: cout << "state: halt " << msg << endl; return;
	}
}

void Control::move()
{
	if(robot.isInside(objectPos)) {
		state = state_push;
		//reset translate PID
		resetPID();
		return;
	}


	//move robot to object
	Point target = objectPos;

	if (abs(objectPos.x - objectWayPoint.x) > abs(objectPos.y - objectWayPoint.y)) {
		if (abs(robot.center.y - target.y) < 20) {
			target.y = robot.center.y;
		}	else {
			target.x = robot.center.x;
		}
	}	else {
		if (abs(robot.center.x - target.x) < 20) {
			target.x = robot.center.x;
		}	else {
			target.y = robot.center.y;
		}
	}

	int angle = getAngle(target - robot.center);

	if (abs(robot.angle - angle) % 360 > 60 ) {
		//rotate to angle
		rotate(angle);
		return;
	}

	translate(target);
}

void Control::push()
{
	Point2d alignment = uvector(objectPos - objectWayPoint) + uvector(objectPos - robot.center);
	Point target = objectPos;
	Kp = Kd = 0;
	Ki = 0.1;
	if (norm(alignment) < 0.1) {
	// if robot and push point are not aligned
		target = objectWayPoint;
		int angle = getAngle(objectWayPoint - objectPos);
		
		if (abs(robot.angle - angle) % 360 > 60) {
			rotate(angle);
			return;
		}
	}
	
	if (norm(robot.center - target) < 10) {
		state = state_halt;
		msg = "Mission complete.";
	}

	if (maxForceCounter > 4) {
		state = state_halt;
		maxForce = robot.force;
		msg = "Maximum force applied 4 times.";
	}

	if (robot.force >= maxForce) {
		maxForceCounter++;
		//coilDriver->send(MINPOWER * -sign(x_coil), MINPOWER * -sign(y_coil));
		coilDriver->send(0,0);
		resetPID();
		return;
	}

	translate(target);
}

void Control::translate(Point target) {
	cout << "Translate from " << robot.center << " to " << target << endl;

	Point2d error = target - robot.center;
	pwm = Kp * error.x;
	pwm += Kd * (error - last_error).x;
	pwm += Ki * error_sum.x;
	pwm = pwm > MAXPOWER ? MAXPOWER : pwm;
	pwm = pwm < -MAXPOWER ? -MAXPOWER : pwm;
	x_coil = pwm;

	pwm = Kp * error.y;
	pwm += Kd * (error - last_error).y;
	pwm += Ki * error_sum.y;
	pwm = pwm > MAXPOWER ? MAXPOWER : pwm;
	pwm = pwm < -MAXPOWER ? -MAXPOWER : pwm;
	y_coil = pwm;

	cout << "Coil PWM:" << x_coil << " " << y_coil << endl;

	coilDriver->send(x_coil, y_coil);

	last_error = error;
	error_sum += error;
}


void Control::rotate(int angle)
{
	//positive CCW, negative CW
	angle = (angle < 1) ? angle + 360 : angle;
	int dir;
	int angle_delta = angle - robot.angle;
	if (angle_delta > 0) {
		dir = 1;
		if (angle_delta > 180)
			dir = -1;
	}
	else {
		dir = -1;
		if (angle_delta < -180)
			dir = 1;
	}
	//direction = (angle - robot.angle) > 180 ? -1 : 1;

	int nextAngle = robot.angle + 30 * dir;
	cout << "Rotate from " << robot.angle << " to " << nextAngle << " desired angle is " << angle << endl;
	x_coil = cos(nextAngle* PI / 180.0) * MINPOWER;
	y_coil = -sin(nextAngle * PI / 180.0) * MINPOWER;
	coilDriver->send(x_coil, y_coil);
	cout << "Coil PWM:" << x_coil << " " << y_coil << endl;

	//reset translate PID
	resetPID();
}


Control::~Control()
{
}
