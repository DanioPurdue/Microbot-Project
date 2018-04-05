
#if !defined FTRACKER
#define FTRACKER

#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2\cudafeatures2d.hpp>
#include <ctime>

#include "videoprocessor.h"

//#include "videoprocessorVideo.h"
#include "Control.h"
#include "CoilDriver.h"
#include "MicroObject.h"
#define CALIBRATIONFRAMENUM 5

#define PI 3.14159265

using namespace std;		//cuda

class FeatureTracker : public FrameProcessor {

	cv::Mat gray;			// current gray-level image
	std::vector<cv::Point2f> points[2]; // tracked features from 0->1
	double minDist;   // minimum distance between two feature points
	std::vector<uchar> status; // status of tracked features
	std::vector<float> err;    // error in tracking
	std::vector<int> forceXHist;
	

	int testCounter;	//this count how many frames have lasted since the start of the tracking

	//Closed Loop Object
	Control *control;
	CoilDriver *coilDriver;

	/*surf*/
public:
	/*calibrationCounter and calibrationDistanceY are used to record the
	distance between the probe and the body*/
	int calibrationCounter;
	int calibrationDistanceY;
	int calibrationDistanceX;
	int histPosition;
	int histOrientation;
	int bodyLengthInPixel;
	int probeAndRobotCenterDistiance;
	
	int robotWidthInPixel;
	Point targetLocation;
	Point targetDirection;


	struct Force2D {
		int forceX;
		int forceY;
	};

	//write force information
	Mat img_matches;
	//this show the location of the body and the probe
	struct LocationInfo{
		Point matchLoc;
		int probeLocX = 0;
		int probeLocY = 0;
		int bodyLocY = 0;	//body location only have information in y
		int boydLocX = 0;

		//Orientation
		int angle = 0; //Degree

		//these are for the size of the probe
		int probeRows = 0;
		int probeCols = 0;
	};
	Point bodyCenter;	//is the center of the body in the Template matching frame

	//speed up for IO
	Mat templProbe;
	Mat templBodyOnly;

	//speed up the for the surf detection
	//the object only needs to be calculated once
	bool hasBodySurfed;
	cv::cuda::GpuMat descriptorsObjectGPU, keypointsObjectGPU;
	cv::cuda::GpuMat img_object_gpu;
	Point roiOffset;	//this is the midpoint of the robot in the scene
	int orientationAngle;
	int angle;


	Mat img_object; //store the feature points information of the object
	std::vector<cv::KeyPoint> keypoints_object;

	//Kalman Filter
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
	bool kalmanInit;
	
	//Export Force Data
	ofstream file;

	FeatureTracker() : minDist(10.) {
		testCounter = 0;

		//different counter
		calibrationCounter = 0;
		calibrationDistanceY = 0;
		histPosition = 0;
		histOrientation = 0;
		bodyLengthInPixel = -1;
		hasBodySurfed = false;
		bodyCenter = Point(0, 0);
		kalmanInit = false;
		roiOffset = Point(0, 0);

		coilDriver = new CoilDriver("COM3");    // adjust as needed
		control = new Control(coilDriver);
	}

	std::vector<cv::KeyPoint> keypoints_scene;
	std::vector<cv::DMatch > matches;

	void setRobotWidth(int width)
	{
		robotWidthInPixel = width;
	}

	void setTargetInformation(Point targetLoc, Point targetDirec)
	{
		targetLocation.x = targetLoc.x;
		targetLocation.y = targetLoc.y;
		targetDirection.x = targetDirec.x;
		targetDirection.y = targetDirec.y;
	}

	// processing method
	void process(cv::Mat &frame, cv::Mat &output) {
		clock_t start, end;
		start = clock();

		//Testing
		testCounter++;
		std::cout << "Test Counter: " << testCounter << std::endl;

		// convert to gray-level image
		//surf
		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		Mat surfImage;
		gray.copyTo(surfImage);
		surfTracker2(gray, "./image/body.JPG");

		//hough transform
		//frame.copyTo(output); //if you delete this line there will be an exeception

		clockTime(start, end, "overall");
		cout << endl << endl;
	}

	//print out the clock time
	void clockTime(clock_t & start, clock_t& end, string processPart)
	{
		end = clock();
		double fps = 1 / ((double(end - start) / CLOCKS_PER_SEC));
		//std::cout << processPart << "process took " << fps << " fps" <<'\n';
		std::cout << processPart << " process took " << 1 / fps << " seconds" << endl;
	}

	// preprocess the image to make it easier for detection
	void removeTheDust(cv::Mat &frame)
	{
		//cv::Mat structElement1(3, 3, CV_8UC1, cv::Scalar(1));
		Mat structElement1 = getStructuringElement(CV_SHAPE_ELLIPSE, Size(12, 12));
		cv::morphologyEx(frame, frame, cv::MORPH_OPEN, structElement1);
		cv::threshold(frame, frame, 75, 230, CV_THRESH_BINARY);
	}

	void showImage(cv::Mat image, char* frameTitle)
	{
		cv::namedWindow(frameTitle);
		cv::imshow(frameTitle, image);
		cv::waitKey(1);
	}

	//this function returns the absolute difference between two points
	double difference(cv::Point refPoint, cv::Point currPoint)
	{
		return abs(refPoint.x - currPoint.x) + abs(refPoint.y - currPoint.y);
	}

	//surf tracker
	void surfTracker2(cv::Mat &grayImage, string fileName)
	{
		clock_t start, end;
		start = clock();

		if (testCounter < 2)
		{
			img_object = cv::imread(fileName, CV_LOAD_IMAGE_GRAYSCALE); //read in the targeT image
		}

		Mat img_scene;
		grayImage.copyTo(img_scene); //read the entire image

		if (!img_object.data || !img_scene.data)
		{
			std::cout << " --(!) Error reading images " << std::endl;
			return;
		}

		//-- Step 1: Detect the keypoints using SURF Detector
		int minHessian = 400;
		int min_dist = 100;

		//===============================================================================================
		cv::cuda::GpuMat imgScene;
		if (testCounter < 2) // it could be one
		{
			img_object_gpu.upload(img_object);
			CV_Assert(!img_object_gpu.empty());
		}

		imgScene.upload(img_scene);
		CV_Assert(!imgScene.empty());



		//Printing the device information
		//cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
		clockTime(start, end, "GPU Before surf");

		cv::cuda::SURF_CUDA surf(400);
		// detecting keypoints & computing descriptors
		cv::cuda::GpuMat keypointsSceneGPU, descriptorsSceneGPU;
		if (testCounter < 2) // it could be one
		{
		surf(img_object_gpu, cv::cuda::GpuMat(), keypointsObjectGPU, descriptorsObjectGPU);
		}
		surf(imgScene, cv::cuda::GpuMat(), keypointsSceneGPU, descriptorsSceneGPU);
		
		clockTime(start, end, "GPU Before Matching");
		// matching descriptors
		cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
		vector<vector<cv::DMatch>> matches;
		matcher->knnMatch(descriptorsObjectGPU, descriptorsSceneGPU, matches, 2);

		clockTime(start, end, "GPU Before Download");
		// downloading results
		vector<float> descriptors_object, descriptorsScene;

		surf.downloadKeypoints(keypointsObjectGPU, keypoints_object);
		surf.downloadKeypoints(keypointsSceneGPU, keypoints_scene);
		surf.downloadDescriptors(descriptorsObjectGPU, descriptors_object);
		surf.downloadDescriptors(descriptorsSceneGPU, descriptorsScene);

		//===============================================================================================
		clockTime(start, end, "GPU After download");
		//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
		std::vector< cv::DMatch > good_matches;

		//if the good feature points are less than 10, skip this frame
		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i][0].distance < 3 * min_dist)
			{
				good_matches.push_back(matches[i][0]);
			}
		}
		
		//Testing Showing the number of feature points
		//std::cout << "Number of feature points: " << good_matches.size() << std::endl;

		img_matches = img_scene;
		/*drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
			good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);*/

		//-- Localize the object
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (int i = 0; i < good_matches.size(); i++)
		{
			//-- Get the keypoints from the good matches
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

		if (obj.size() < 4)
		{
			std::cout << "Skipped a frame (in homography)" << std::endl;
			return;
		}
		Mat H = findHomography(obj, scene, cv::RANSAC);

		//-- Get the corners from the image_1 ( the object to be "detected" )
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(img_object.cols, 0);
		obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
		obj_corners[3] = cvPoint(0, img_object.rows);

		clockTime(start, end, "GPU Homography");
		std::vector<Point2f> scene_corners(4);
		perspectiveTransform(obj_corners, scene_corners, H);

		//===============================================================================================

		//if points are clustered together
		//check this part  
		if (orientationCheck(scene_corners[0], scene_corners[1]) == false)
		{
			return;
		}

		//////Top image
		//line(img_matches, scene_corners[0], scene_corners[1] , Scalar(0, 255, 0), 4);
		////Right image
		//line(img_matches, scene_corners[1], scene_corners[2] , Scalar(0, 255, 0), 4);
		////Bottom image
		//line(img_matches, scene_corners[2] , scene_corners[3] , Scalar(0, 255, 0), 4);
		////Left image
		//line(img_matches, scene_corners[3], scene_corners[0] , Scalar(0, 255, 0), 4);


		Point2f midPoint = getMidPoint(scene_corners[0], scene_corners[2]);
		roiOffset.x = (int)midPoint.x;
		roiOffset.y = (int)midPoint.y;

		//cv::namedWindow("Good Matches & Object detection");
		////-- Show detected matches
		//imshow("Good Matches & Object detection", img_matches);
		//write the image to the scene
		/*waitKey(1);*/
		//=============================Rotated Image ROI==============================
		// rect is the RotatedRect
		//Calculate Angle and size
		double lengthOfRectangle = difference(scene_corners[1], scene_corners[2]);
		double widthOfRectangle = difference(scene_corners[0], scene_corners[1]);
		double RecAngle = atan2(scene_corners[0].y - scene_corners[1].y, scene_corners[0].x - scene_corners[1].x) * 180 / 3.1415926;
		double RecAngle03 = atan2(scene_corners[3].y - scene_corners[0].y, scene_corners[3].x - scene_corners[0].x) * 180 / 3.1415926;
		
		//checking whether a rectangle has been formed
		double testRecAngle;
		double testRecAngle03;
		if (RecAngle < 0)
		{
			testRecAngle = 360 + RecAngle;
		}
		else
		{
			testRecAngle = RecAngle;
		}

		if (RecAngle03 < 0)
		{
			testRecAngle03 = 360 + RecAngle03;
		}
		else
		{
			testRecAngle03 = RecAngle03;
		}
		if (abs(testRecAngle - 90 - testRecAngle03) > 15)
		{
			//std::cerr << "RecAngle: " << RecAngle << std::endl;
			//std::cerr << "RecAngle03: " << RecAngle03 << std::endl;
			std::cerr << "invalid surf rectangle" << std::endl;
			imshow("Good Matches & Object detection", img_matches);
			return;
		}


		//////Top image
		//cv::line(img_matches, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 4);
		////Right image
		//cv::line(img_matches, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 4);
		////Bottom image
		//cv::line(img_matches, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 4);
		////Left image
		//cv::line(img_matches, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 4);
		//
		orientationAngle = 180 - RecAngle;
		if (orientationAngle > 180)
		{
			orientationAngle = orientationAngle - 360;
		}
		orientationAngle = -orientationAngle; //reverse the sign of the angle

		cv::RotatedRect rect = cv::RotatedRect(midPoint, cv::Size2f(img_object.cols, img_object.rows), RecAngle);
		angle = RecAngle;
		
		// matrices we'll use
		Mat M, rotated, cropped;
		// get angle and size from the bounding box
		float angle = rect.angle;
		Size rect_size = rect.size;

		// get the rotation matrix
		int calibrationAngle = 3;
		M = getRotationMatrix2D(rect.center, angle + 180, 1.0);
		

		clockTime(start, end, "before warpAffine");
		cv::warpAffine(img_scene, rotated, M, img_matches.size(), cv::INTER_CUBIC);
		// crop the resulting image
		getRectSubPix(rotated, rect_size, rect.center, cropped);


		clockTime(start, end, "before template matching");
		//call the template matching method
		callTemplateMatching(cropped);


		cv::namedWindow("Good Matches & Object detection");
		//-- Show detected matches
		imshow("Good Matches & Object detection", img_matches);

		cv::waitKey(1);
	}


	cv::Point2f getMidPoint(cv::Point2f a, cv::Point2f b)
	{

		float x = (a.x + b.x) / 2;
		float y = (a.y + b.y) / 2;
		return Point2f(x, y);
	}

	void callTemplateMatching(cv::Mat img)
	{
		//===================================Probe Location============================================
		Mat result;
		if (testCounter < 2) //this is only read for the first two frames
		{
			templProbe = cv::imread("./image/probe.JPG", CV_LOAD_IMAGE_GRAYSCALE); //read in the targe image
			templBodyOnly = cv::imread("./image/bodyOnly.JPG", CV_LOAD_IMAGE_GRAYSCALE); //read in the targe image
		}
		int match_method = 0; //there are 6 different modes from 0 to 5
		struct LocationInfo locationInfo = matchingMethodProbe(0, 0, img, templProbe, result, match_method);

		//==================================Body Only===================================================
		cv::Mat resultBodyOnly;
		cv::Point bodyLoc =  matchingMethodBody(0, 0, img, templBodyOnly, resultBodyOnly, match_method);
		
		cv::circle(img, bodyLoc, 32.0, cv::Scalar(255, 255, 255), 1, 8);
		locationInfo.bodyLocY = bodyLoc.y;
		locationInfo.boydLocX = bodyLoc.x;
		writeForceInformation(img, locationInfo.matchLoc, locationInfo.probeRows, locationInfo.probeCols, locationInfo.probeLocX, locationInfo.probeLocY, locationInfo.boydLocX, locationInfo.bodyLocY);
		
		//update the body center
		bodyCenter = bodyLoc;
		probeAndRobotCenterDistiance = (int)(img.rows / 2) - locationInfo.probeLocY;
	}

	struct LocationInfo matchingMethodProbe(int, void*, Mat img, Mat templ, Mat result, int match_method)
	{
		/// Source image to display
		cv::Mat img_display;
		img.copyTo(img_display);

		/// Create the result matrix
		int result_cols = img.cols - templ.cols + 1;
		int result_rows = img.rows - templ.rows + 1;

		result.create(result_rows, result_cols, CV_32FC1);

		/// Do the Matching and Normalize
		matchTemplate(img, templ, result, match_method);
		cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

		/// Localizing the best match with minMaxLoc
		double minVal; double maxVal; Point minLoc; Point maxLoc;
		cv::Point matchLoc;

		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
		if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
		{
			matchLoc = minLoc;
		}
		else
		{
			matchLoc = maxLoc;
		}

		struct LocationInfo locationInfo;

		locationInfo.probeLocY = matchLoc.y + templ.rows / 2;
		locationInfo.probeLocX = matchLoc.x + templ.cols / 2;


		locationInfo.probeRows = templ.rows;
		locationInfo.probeCols = templ.cols;
		locationInfo.matchLoc.x = matchLoc.x;
		locationInfo.matchLoc.y = matchLoc.y;

		
		return locationInfo;
	}


	Point matchingMethodBody(int, void*, Mat img, Mat templ, Mat result, int match_method)
	{
		/// Source image to display
		cv::Mat img_display;
		img.copyTo(img_display);

		/// Create the result matrix
		int result_cols = img.cols - templ.cols + 1;
		int result_rows = img.rows - templ.rows + 1;

		result.create(result_rows, result_cols, CV_32FC1);

		/// Do the Matching and Normalize
		cv::matchTemplate(img, templ, result, match_method);
		cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

		/// Localizing the best match with minMaxLoc
		double minVal; double maxVal; Point minLoc; Point maxLoc;
		Point matchLoc;

		cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

		/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
		if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
		{
			matchLoc = minLoc;
		}
		else
		{
			matchLoc = maxLoc;
		}

		matchLoc.y = matchLoc.y + templ.rows / 2;
		matchLoc.x = matchLoc.x + templ.cols / 2;

		return matchLoc;
	}


	void writeForceInformation(Mat& img_display, Point matchLoc ,int probeRows, int probeCols,int probeLocX,int probeLocY, int bodyLocX, int bodyLocY)
	{
		
		if (testCounter > 5)
		{
			cv::Point probeLocCorrect = probeStablizer(probeLocX, probeLocY, bodyLocX, bodyLocY);
			probeLocX = probeLocCorrect.x;
			probeLocY = probeLocCorrect.y;
			
			matchLoc.x = probeLocY - probeRows / 2;
			matchLoc.x = probeLocX - probeCols / 2;
		}

		//Testing show the location of the probe 
		circle(img_display, Point2f(probeLocX, probeLocY), 32.0, cv::Scalar(255, 0, 255), 1, 8);
		//
	

		struct Force2D force2D = ForceCalculator(bodyLocX, bodyLocY, probeLocX, probeLocY, img_display.rows * 2 / 3);
		
		
		int offset = 20;
		///result image
		std::stringstream ss1;
		ss1 << "Orientation[Degree]: " << orientationAngle << " BodyX: " << roiOffset.x << " BodyY: " << roiOffset.y;
		putText(img_matches, ss1.str(), Point(3, img_matches.rows / 8 * 7.1 - offset), cv::FONT_HERSHEY_TRIPLEX, 1.3, cv::Scalar(0, 0, 255));
		
		cv::Point probeLocation = findProbeLocation(orientationAngle, roiOffset);
		std::stringstream ss2;
		ss2 << "ProbeX Loc: " << probeLocation.x << " ProbeY Loc: " << probeLocation.y << " Frame Num: " << testCounter;
		putText(img_matches, ss2.str(), Point(3, img_matches.rows / 8 * 7.4 - offset), cv::FONT_HERSHEY_TRIPLEX, 1.3, cv::Scalar(0, 0, 255));
		
		//Testing show the location of the probe 
		//probeLocation = KalmanStablizer(probeLocation);
		//circle(img_matches, probeLocation, 20.0, Scalar(255, 255, 255), 1, 8);
		//circle(img_matches, roiOffset, 32.0, Scalar(255, 255, 255), 1, 8);
		//

		std::stringstream ss3;
		ss3 << "Y Component Force[microN]: " << abs(force2D.forceY);
		putText(img_matches, ss3.str(), Point(3, img_matches.rows / 8 * 7.7 - offset), cv::FONT_HERSHEY_TRIPLEX, 1.3, cv::Scalar(0, 0, 255));

		std::stringstream ss4;

		ss4 << "X Component Force[microN]: " << force2D.forceX;
		putText(img_matches, ss4.str(), Point(3, img_matches.rows / 8 * 8 - offset + 5), cv::FONT_HERSHEY_TRIPLEX, 1.3, cv::Scalar(0, 0, 255));

		
		//======Export Data======
		//Write Video
		if (testCounter == 1)
		{
			file.open("./image/forceData.txt");
			file << "frameCounter\tforce x\tforce y" << endl;
		}
		else
		{
			file << testCounter << "\t" << force2D.forceX << "\t" << abs(force2D.forceY) << endl;
		}
		
		//
		if (testCounter > 5) //update the robot info
		{
			control->update(MicroObject(Point(roiOffset.x, roiOffset.y), robotWidthInPixel, orientationAngle, force2D.forceY));
		}
		else if (testCounter == 5) //set the target info
		{
			control->setTarget(targetLocation, targetDirection);
		}
		imshow("Template Matching", img_display);
		return;

	}

	Point findProbeLocation(int orientation, Point bodyLocation)
	{
		int locationY = -cos(orientation * PI / 180) * abs(probeAndRobotCenterDistiance);
		int locationX = sin(orientation * PI / 180) * abs(probeAndRobotCenterDistiance);
		Point probeLocation(locationX + bodyLocation.x, locationY + bodyLocation.y);
		return probeLocation;

	}

	struct Force2D ForceCalculator(int probeLocX, int probeLocY, int bodyLocX, int bodyLocY, int framePixelLength)
	{
		double forces[2] = { 0, 0 };
		struct Force2D force2D;
		force2D.forceX = 0;
		force2D.forceY = 0;

		int distanceY = bodyLocY - probeLocY;
		int distanceX = probeLocX - bodyLocX;
		if (calibrationCounter < CALIBRATIONFRAMENUM)
		{
			calibrationDistanceY += distanceY;
			calibrationDistanceX += distanceX;
			histPosition += probeLocY;
			calibrationCounter++;
			bodyLengthInPixel += framePixelLength;
		}
		else if (calibrationCounter == CALIBRATIONFRAMENUM)
		{
			calibrationDistanceY = calibrationDistanceY / CALIBRATIONFRAMENUM;
			calibrationDistanceX = calibrationDistanceX / CALIBRATIONFRAMENUM;
			histPosition = histPosition / CALIBRATIONFRAMENUM;
			bodyLengthInPixel = bodyLengthInPixel / CALIBRATIONFRAMENUM;
			calibrationCounter++;
		}
		else
		{
			double slopeX = 0.302; //this is the slope of the curve fit line that converts distance to force
			double slopeY = 0.105;
			double bodyLength = 840; //the value for the new robot
			double lengthToPixelRatio = bodyLength / robotWidthInPixel;
			//cout << "++++++++++++++++++++++robotWidth: " << robotWidthInPixel << endl;
			//============================Y distance: Vertical Component of the force===================================
			double deformedDistanceY = (calibrationDistanceY - distanceY) * lengthToPixelRatio;	//conversion from pixel to force
			force2D.forceY = slopeYFunction(deformedDistanceY);
			//this for the stabilization of the robot
			//if (abs(force2D.forceY) <= 2)	//this is added to eliminate the noise
			//{
			//	force2D.forceY = 0;
			//}

			//============================X distance: Horizontal Component of the force===================================
			double deformedDistanceX = (calibrationDistanceX - distanceX) * lengthToPixelRatio;	//conversion from pixel to force
			force2D.forceX = slopeXFunction(deformedDistanceX);
			if (abs(force2D.forceX) <= 1 || force2D.forceY == 0)
			{
				force2D.forceX = 0;
			}

			//Testing
			std::cout << "LengthToPixel:" << lengthToPixelRatio << std::endl;
			std::cout << "Testing 2: " << distanceY << " Calibration Distance[y]: " << calibrationDistanceY << " Deformation[y]: " << calibrationDistanceY - distanceY << std::endl;
			//

		}

		return force2D;
	}


	//pixel to force conversion function
	double slopeYFunction(double deformationDistance)
	{
		return 0.0014 * pow(deformationDistance, 2) + 0.1024 * deformationDistance - 0.0889;
	}

	double slopeXFunction(double deformationDistance)
	{
		return 0.035 * deformationDistance - 0.6227;
	}

	int distanceStablizer(int distance)
	{
		if (calibrationCounter > CALIBRATIONFRAMENUM)
		{
			if (abs(distance - histPosition) >= 4)
			{
				//if the distance too much use the history position
				distance = histPosition;
				return distance;
			}
			else
			{
				//update history position
				histPosition = distance;
				//Testing
				std::cout << "history update: " << histPosition << std::endl;
			}
		}
		return distance;
	}

	//this check the validity of the surf mathcher, if the points come acrossed each other, the frame should be skipped 
	bool orientationCheck(Point2f a, Point2f b)
	{
		if (abs(a.x - b.x) < 4 && abs(a.y - b.y) < 4)
		{
			std::cout << "Skipped a frame" << std::endl;
			return false;
		}
		return true;
	}


	Point ROIPoint(Point robotCenterAbsScale)
	{
		if (robotCenterAbsScale.x - img_object.cols * 3 / 4 < 0)
		{
			robotCenterAbsScale.x = 1;
		}
		else
		{
			robotCenterAbsScale.x = robotCenterAbsScale.x - img_object.cols * 3 / 4;
		}

		if (robotCenterAbsScale.y - img_object.cols * 3 / 4 < 0)
		{
			robotCenterAbsScale.y = 1;
		}
		else
		{
			robotCenterAbsScale.y = robotCenterAbsScale.y - img_object.cols * 3 / 4;
		}

		return robotCenterAbsScale;
	}

	Point ROISize(Point roiPoint, int sceneRows, int sceneCols)
	{
		int width = img_object.cols * 6.5 / 4;
		int length = img_object.rows * 6.5 / 4;
		
		if ((width + roiPoint.x) > sceneCols)
		{
			width = sceneCols - roiPoint.x - 1;
		}

		if ((length + roiPoint.y) > sceneRows)
		{
			length = sceneRows - roiPoint.y - 1;
		}
		return Point(width, length);
	}


	//this is currently not used for tracking of the robe, as it is not very sensitive to sudden displacement
	Point KalmanStablizer(Point probeBodyDiff)
	{
		if (kalmanInit == false)
		{
			KF = cv::KalmanFilter(4, 2, 0);	//class variable
			KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
			measurement = cv::Mat_<float>(2, 1); //class variable
			measurement.setTo(cv::Scalar(0));


			KF.statePre.at<float>(0) = probeBodyDiff.x;
			KF.statePre.at<float>(1) = probeBodyDiff.y;
			KF.statePre.at<float>(2) = 0;
			KF.statePre.at<float>(3) = 0;


			setIdentity(KF.measurementMatrix);
			setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
			setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
			setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
			
			kalmanInit = true;
		}

		// First predict, to update the internal statePre variable
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

		// Get body Position
		measurement(0) = probeBodyDiff.x;
		measurement(1) = probeBodyDiff.y;

		// The update phase 
		Mat estimated = KF.correct(measurement);
		Point statePt(estimated.at<float>(0), estimated.at<float>(1));
		return statePt;
	}

	Point probeStablizer(int probeLocX, int probeLocY, int bodyLocX, int bodyLocY)
	{
		if ((bodyLocY - probeLocY) < (-1 * calibrationDistanceY / 2)) // if it is smaller than 50% of the calibration distance, the tracking method is wrong
		{
			probeLocY = bodyLocY + calibrationDistanceY;
		}

		if (abs(bodyLocX - probeLocX) > 20 )
		{
			probeLocX = bodyLocX - calibrationDistanceX;
		}

		return Point(probeLocX, probeLocY);
	}

	int forceXStack(int forceX)
	{
		if (forceXHist.size() < 3)
		{
			forceXHist.push_back(forceX);
			return forceX;
		}


		int averageForce = ( forceXHist[0] + forceXHist[1] + forceXHist[2] ) / 3;
		forceXHist[0] = forceXHist[1];
		forceXHist[1] = forceXHist[2];
		forceXHist[2] = forceX;
		
		if (abs(averageForce - forceX) > 5)
		return averageForce;
		
		return forceX;
		
	}
};



#endif
