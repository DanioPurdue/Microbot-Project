#pragma once
#include <opencv2/core/core.hpp>

class MicroObject
{
public:
	cv::Point center;
	int radius;
	int angle;
	float force;
	MicroObject(cv::Point center, int radius, int angle, float force);
	MicroObject();
	bool isInside(MicroObject that);
	bool isInside(cv:: Point pt);
	bool isTouched(MicroObject that);
	bool isTouched(cv::Point pt);
	~MicroObject();
};

