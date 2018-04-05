#define DEBUG true
#include "Track.h"
#include "Particle.h"

using namespace cv;

Track::Track()
{
	robot = imread("robot1.jpg ");
	object = imread("object3.jpg");

	surf_detector = SurfFeatureDetector(minHessian);
	surf_detector.detect(robot, keypoints_robot);
	surf_detector.detect(object, keypoints_object);

	extractor.compute(robot, keypoints_robot, descriptors_robot);
	extractor.compute(object, keypoints_object, descriptors_object);

	robot_corners = vector<Point2f>(4);
	robot_corners[0] = cvPoint(0, 0);
	robot_corners[1] = cvPoint(robot.cols, 0);
	robot_corners[2] = cvPoint(robot.cols, robot.rows);
	robot_corners[3] = cvPoint(0, robot.rows);

	object_corners = vector<Point2f>(4);
	object_corners[0] = cvPoint(0, 0);
	object_corners[1] = cvPoint(object.cols, 0);
	object_corners[2] = cvPoint(object.cols, object.rows);
	object_corners[3] = cvPoint(0, object.rows);
}

Particle Track::find(Mat descriptors_target, Mat descriptors_frame, vector<KeyPoint> keypoints_target, vector<KeyPoint> keypoints_frame, vector<Point2f> tar_corners) {
	Particle target;
	vector<vector<cv::DMatch> > matches_target;
	bf_matcher.knnMatch(descriptors_target, descriptors_frame, matches_target, 2);

	//-- Draw only "good" matches
	std::vector< DMatch > good_matches;

	for (size_t i = 0; i < matches_target.size(); ++i)
	{
		if (matches_target[i].size() < 2) continue;

		if (matches_target[i][0].distance <= 0.7 * matches_target[i][1].distance)
			good_matches.push_back(matches_target[i][0]);
	}

	//-- Localize the object
	std::vector<Point2f> tar;
	std::vector<Point2f> fra;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		tar.push_back(keypoints_target[good_matches[i].queryIdx].pt);
		fra.push_back(keypoints_frame[good_matches[i].trainIdx].pt);
	}

	if (tar.size() < 4) return target;


	Mat H = findHomography(tar, fra, CV_RANSAC);

	//-- Get the corners from the image_1 ( the object to be "detected" )

	std::vector<Point2f> fra_corners(4);
	
	perspectiveTransform(tar_corners, fra_corners, H);
	Point2f angle_point = fra_corners[1] - fra_corners[0];

	int angle = atan(angle_point.y / angle_point.x) * 180 / 3.1416;

	if (angle_point.y < 0 && angle_point.x < 0)
		angle = 180 - angle;
	if (angle_point.y > 0 && angle_point.x < 0)
		angle = 180 + angle;
	if (angle_point.x > 0)
		angle = -angle;

	target.radius = 0.5 * norm(fra_corners[0] - fra_corners[2]);
	target.center = (fra_corners[0] + fra_corners[2]) * 0.5f;
	target.angle = angle;

	return target;
}



void Track::update(Mat frame) {

	//<KeyPoint> keypoints_frame;
	Mat descriptors_frame;

	//Detect the keypoints from frame
	surf_detector.detect(frame, keypoints_frame);
	extractor.compute(frame, keypoints_frame, descriptors_frame);

	//Particle robot = find(descriptors_robot, descriptors_frame, keypoints_robot, keypoints_frame, robot_corners);
	//Particle object = find(descriptors_object, descriptors_frame, keypoints_object, keypoints_frame, object_corners);	
	addPos(find(descriptors_robot, descriptors_frame, keypoints_robot, keypoints_frame, robot_corners), robot_pos);
	addPos(find(descriptors_object, descriptors_frame, keypoints_object, keypoints_frame, object_corners), object_pos);
/*
	if (DEBUG) {
		Mat imgMatches;
		cv::drawMatches(robot, keypoints_robot, frame, keypoints_frame, good_matches, imgMatches);
		imshow("SURF matches", imgMatches);
		cout << good_matches.size() << endl;
	}
*/  
	cout << robot_pos.size() << endl;
	Particle robot = robot_pos.back();
	stringstream sx, sy, sAngle;
	///CV_CAP_PROP_FPS for frame count
	circle(frame, robot.center, robot.radius, Scalar(0, 255, 0), 1, 8, 0);
	sx << "x:" << robot.center.x;
	sy << "y:" << robot.center.y;// << " " << frameNum;
	sAngle << "Angle: " << robot.angle;
	string x_cord = sx.str();
	string y_cord = sy.str();
	string theta = sAngle.str();
	putText(frame, x_cord.c_str(), cv::Point(750, 55),
		FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(frame, y_cord.c_str(), cv::Point(750, 85),
		FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(frame, theta.c_str(), cv::Point(750, 115),
		FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));

	imshow("Original", frame);
	keypoints_frame.clear();
	//There is a bug.
	//keypoints_frame.shrink_to_fit();
}

void Track::addPos(Particle &target, vector<Particle> &target_pos)
{
	/*
	if(target_pos.size() > 1)
		int avr_radius = (target_pos[target_pos.size()].radius + target_pos[target_pos.size() - 1].radius) *0.5;
	if (target.radius > 0) {
		if (target_pos.size() < 2) {
			target_pos.push_back(target);
			return;
		}
		else if (target.radius > avr_radius * 0.7 && target.radius < avr_radius * 01.3) {
			target_pos.push_back(target);
			return;
		}			
	}
	target.radius = avr_radius;
	target.center = target_pos.back().center + target_pos.back().center - target_pos[target_pos.size() - 1].center;
	*/
	target_pos.push_back(target);
	cout << "pos added" << endl;
	cout << target_pos.size() << endl;
}

Track::~Track()
{
}
