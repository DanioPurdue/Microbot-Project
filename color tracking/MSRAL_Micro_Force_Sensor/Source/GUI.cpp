#include "GUI.h"


bool GUI::update(string imgName)
{
	if (point.size() == 4) {
		Mat cropped = getTransmtx(point);
		imshow(imgName, cropped);
		imwrite("./image/" + imgName +".jpg", cropped);
		return true;
	}  

	if (!point.empty()) {
		line(*frame, point.back(), ptr, cv::Scalar(255, 0, 0), 1, 8);
		if(point.size() == 3)
			line(*frame, point.front(), ptr, cv::Scalar(255, 0, 0), 1, 8);

		for (int i = 0; i < point.size(); i++) {
			if (i > 0) {
				line(*frame, point[i - 1], point[i], cv::Scalar(255, 0, 0), 1, 8);
			}
			circle(*frame, point.at(i), 2, cv::Scalar(0, 255, 0), 1, 8);
		}
	}
	return false;
}

void GUI::robotWidth(int* width )
{
	if (point.size() == 2) {
		for (int i = 0; i < point.size(); i++) {
			if (i > 0) {
				line(*frame, point[i - 1], point[i], cv::Scalar(255, 0, 0), 1, 8);
			}
			circle(*frame, point.at(i), 2, cv::Scalar(0, 255, 0), 1, 8);
		}

		*width = findDistance(point.at(0), point.at(1));
		cout << "Robot width in pixel: " << *width << endl;
		point.clear();
		return;
	}

	if (!point.empty()) {
		line(*frame, point.back(), ptr, cv::Scalar(255, 0, 0), 1, 8);
		for (int i = 0; i < point.size(); i++) {
			if (i > 0) {
				line(*frame, point[i - 1], point[i], cv::Scalar(255, 0, 0), 1, 8);
			}
			circle(*frame, point.at(i), 2, cv::Scalar(0, 255, 0), 1, 8);
		}
	}
	return;
}

void GUI::targetInformation(Point* targetLocation, Point* targetDirection)
{
	if (point.size() == 2) {
		for (int i = 0; i < point.size(); i++) {
			if (i > 0) {
				line(*frame, point[i - 1], point[i], cv::Scalar(255, 0, 0), 1, 8);
			}
			circle(*frame, point.at(i), 2, cv::Scalar(0, 255, 0), 1, 8);
		}

		targetLocation->x = (point.at(0)).x;
		targetLocation->y = (point.at(0)).y;
		targetDirection->x = (point.at(1)).x;
		targetDirection->y = (point.at(1)).y;
		
		point.clear();
		return;
	}

	if (!point.empty()) {
		line(*frame, point.back(), ptr, cv::Scalar(255, 0, 0), 1, 8);
		for (int i = 0; i < point.size(); i++) {
			if (i > 0) {
				line(*frame, point[i - 1], point[i], cv::Scalar(255, 0, 0), 1, 8);
			}
			circle(*frame, point.at(i), 2, cv::Scalar(0, 255, 0), 1, 8);
		}
	}
	return;
}

int GUI::findDistance(Point a, Point b)
{
	int distance = sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
	return distance;
}

Mat GUI::getTransmtx(vector<Point> square)
{
	Point2f vertices[4], crop_pts[4];

	cv::RotatedRect rect = minAreaRect(square);
	rect.points(vertices);

	int minDist = 10000;
	int startingVertix = 0;

	for (int i = 0; i < 4; i++) {
		int dist = norm(vertices[i] - (Point2f)square[0]);
		if (dist < minDist) {
			startingVertix = i;
			minDist = dist;
		}
	}

	double width = norm(vertices[(startingVertix + 1) % 4] - vertices[startingVertix]);
	double height = norm(vertices[(startingVertix + 3) % 4] - vertices[startingVertix]);

	Mat cropped = Mat::zeros(Size(width, height), CV_8UC3);

	crop_pts[startingVertix] = Point2f(0, 0);
	crop_pts[(++ startingVertix) % 4] = Point2f(cropped.cols, 0);
	crop_pts[(++ startingVertix) % 4] = Point2f(cropped.cols, cropped.rows);
	crop_pts[(++ startingVertix) % 4] = Point2f(0, cropped.rows);
	
	warpPerspective(*frame, cropped, getPerspectiveTransform(vertices, crop_pts), cropped.size());
	return cropped;
}

GUI::GUI(string frameTitle, Mat* frame)
{	
	this->frame = frame;
	///add listioner to mouse event
	cv::setMouseCallback(frameTitle, callMouseHandler, this);
}


GUI::~GUI()
{
}


void GUI::callMouseHandler(int event, int x, int y, int flags, void * param)
{
	GUI *self = static_cast<GUI*>(param);
	self->mouseHandler(event, x, y, flags);
}

void GUI::mouseHandler(int event, int x, int y, int flags)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		ptr = Point(x, y);
	}

	if (event == CV_EVENT_MOUSEMOVE)
	{
		ptr = Point(x, y);
	}

	if (event == CV_EVENT_LBUTTONUP)
	{
		point.push_back(Point(x, y));
	}

	if (event == CV_EVENT_RBUTTONUP)
	{
		point.clear();
		point.shrink_to_fit();
	}
}
