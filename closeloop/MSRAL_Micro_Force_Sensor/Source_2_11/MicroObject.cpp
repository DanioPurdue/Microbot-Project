#include "MicroObject.h"


MicroObject::MicroObject(cv::Point center, int radius, int angle, float force)
{
	this->center = center;
	this->radius = radius;
	// correct the fucked up angle
	this->angle = (90-angle) % 360;
	if (this->angle < 0)
		this->angle += 360;
	this->force = force;
}

MicroObject::MicroObject()
{
	MicroObject(cv::Point(0,0), 0, 0, 0);
}

bool MicroObject::isInside(MicroObject that)
{
	if (cv::norm(center - that.center) < radius)
		return true;
	else
		return false;
}

bool MicroObject::isInside(cv::Point pt)
{ 
	if (cv::norm(center - pt) < radius)
		return true;
	else
		return false;
}

bool MicroObject::isTouched(MicroObject that)
{
	if (norm(that.center - center) <= 1.1 * (radius + that.radius))
		return true;
	else
		return false;
}

bool MicroObject::isTouched(cv::Point pt)
{
	if (norm(pt - center) <= 1.1 * radius)
		return true;
	else
		return false;
}

MicroObject::~MicroObject()
{
}
