#pragma once


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;
using cv::Mat;
using cv::Point;
using cv::Size;
using cv::Point2f;

#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
#include <vector>

class GUI
{
	Mat* frame;
	void mouseHandler(int event, int x, int y, int flags);
	
public:
	vector<Point> point;
	Point ptr;
	bool update(string imgName);
	Mat getTransmtx(vector<Point>);
	GUI(string frameTitle, Mat* frame);
	~GUI();
	static void callMouseHandler(int event, int x, int y, int flags, void* param);
	void robotWidth(int* width);
	int findDistance(Point a, Point b);
	void targetInformation(Point* targetLocation, Point* targetDirection);
};

