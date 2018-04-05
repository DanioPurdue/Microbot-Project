
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
	int histForceY;
	int bodyLengthInPixel;
	int probeAndRobotCenterDistiance;

	int robotWidthInPixel;
	Point targetLocation;
	Point targetDirection;


	struct Force2D {
		double forceX;
		double forceY;
	};

	struct TwoCntrs {
		Point springCntr;
		Point bodyCntr;
	};

	//write force information
	Mat img_matches;
	//this show the location of the body and the probe
	struct LocationInfo {
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
	cv::cuda::GpuMat descriptorsObject2GPU, keypointsObject2GPU;
	cv::cuda::GpuMat img_object_gpu, img_object2_gpu;
	Point roiOffset;	//this is the midpoint of the robot in the scene
	Point midPointHist;

	//these two counters are used for stabilizing frames
	int histCounter;
	int histCounterAngle;

	int orientationAngle;
	int recAngleHist;
	int angle;


	Mat img_object; //store the feature points information of the object
	Mat img_object2; //template of the alternative.
	std::vector<cv::KeyPoint> keypoints_object;

	//Kalman Filter
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
	bool kalmanInit;

	//Export Force Data
	ofstream file;

	//Color Tracking
	int iLowH;
	int iHighH;

	int iLowS;
	int iHighS;

	int iLowV;
	int iHighV;

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
		midPointHist = Point(0, 0);
		recAngleHist = -800;
		histCounter = 0;
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

		//Colored Image
		Mat colorImage;
		frame.copyTo(colorImage);
		//Point springPos = colorTrackingSpring(colorImage);
		//Point bodyPos = colorTrackingBody(colorImage);

		//colorSamplingHSV(colorImage, targetLocation);
		//colorCalibrationHSV(colorImage);
		// convert to gray-level image
		Point bodyCntr = colorTrackingBody(colorImage);
		Point springCntr = colorTrackingSpring(colorImage);
		struct TwoCntrs twoCntrs;
		twoCntrs.springCntr = springCntr;
		twoCntrs.bodyCntr = bodyCntr;


		//struct TwoCntrs twoCntrs = roi_colorTrackingBody(colorImage);
		Mat surfImage;
		colorImage.copyTo(surfImage);
		surfTracker2(surfImage, "./image/body", twoCntrs.springCntr, twoCntrs.bodyCntr);
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
	//this decide which template form should be used
	//template: 1: the upright template 2 horizontal template
	void surfTracker2(cv::Mat &grayImage, string fileName, Point springPos, Point bodyPos)
	{
		img_matches = grayImage;
		clock_t start, end;
		start = clock();
		if (testCounter < 2)
		{
			img_object = cv::imread(fileName.append(".jpg"), CV_LOAD_IMAGE_GRAYSCALE); //read in the target image
																					   //Rescaled the image to make it compatible with CUDA module
			Mat img_object_bordered; // let border be the same in all directions
			Mat img_object_bordered2;

			int border = img_object.rows * 0.2;

			// form a border in-place
			cv::RNG rng(12345);
			cv::Scalar paddingValue = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255));
			cv::copyMakeBorder(img_object, img_object_bordered, border, border,
				border, border, cv::BORDER_CONSTANT, paddingValue);

			cv::imwrite("./image/borderedTemplate.JPG", img_object_bordered);

			img_object2 = cv::imread("./image/borderedTemplate 3.JPG", CV_LOAD_IMAGE_GRAYSCALE); //<-- you need to create a border template 3
			img_object = img_object_bordered;

			if (!img_object2.data)// Check for invalid input
			{
				cout << "=====================================================================\n" << std::endl;
				return;
			}
		}
		Mat img_scene = grayImage;
		Point2f midPoint = Point((bodyPos.x + springPos.x) / 2, (bodyPos.y + springPos.y) / 2);
		if ((difference(midPoint, midPointHist) <= 20 || (difference(midPoint, midPointHist) >= 80 && midPointHist.x != 0 && midPointHist.y != 0)) && testCounter - histCounter <= 10)
		{
			midPoint = midPointHist;
		}
		else
		{
			midPointHist = midPoint;
			histCounter = testCounter;
		}
		roiOffset.x = (int)midPoint.x;
		roiOffset.y = (int)midPoint.y;

		//=============================Rotated Image ROI==============================
		// rect is the RotatedRect
		//Calculate Angle and size
		double lengthOfRectangle = robotWidthInPixel * 4;
		double widthOfRectangle = robotWidthInPixel * 3;
		double RecAngle = atan2(springPos.y - bodyPos.y, springPos.x - bodyPos.x) * 180 / 3.1415926 - 55; //this is subjet to change 55 degree for the offset one
		cout << "Orientation: " << RecAngle << endl;
		if (abs(RecAngle - recAngleHist) >= 30 && (testCounter - histCounterAngle <= 5) && testCounter > 5) //abs(RecAngle - recAngleHist) >= 1 && (testCounter - histCounterAngle <= 5)
		{
			RecAngle = recAngleHist;
		}
		else
		{
			recAngleHist = RecAngle;
			histCounterAngle = testCounter;
		}
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
		//cv::namedWindow("cropped image");
		//imshow("cropped image", cropped);
		//cv::waitKey(1);

		clockTime(start, end, "before template matching");
		//call the template matching method
		//cv::cvtColor(cropped, cropped, CV_BGR2GRAY);
		//callTemplateMatching(cropped);
		Point probeLoc = colorTrackingProbeBlue(img_scene);
		writeForceInformation(img_matches, robotWidthInPixel, 10, 10, probeLoc.x, probeLoc.y, bodyPos.x, bodyPos.y);
		cv::namedWindow("Good Matches & Object detection");
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
			templProbe = cv::imread("./image/probe.JPG", CV_LOAD_IMAGE_COLOR); //read in the targe image
			templBodyOnly = cv::imread("./image/bodyOnly.JPG", CV_LOAD_IMAGE_COLOR); //read in the targe image
		}
		int match_method = 0; //there are 6 different modes from 0 to 5
		struct LocationInfo locationInfo = matchingMethodProbe(0, 0, img, templProbe, result, match_method);

		//==================================Body Only===================================================
		cv::Mat resultBodyOnly;
		cv::Point bodyLoc = matchingMethodBody(0, 0, img, templBodyOnly, resultBodyOnly, match_method);

		cv::circle(img, bodyLoc, 32.0, cv::Scalar(255, 255, 255), 1, 8);
		locationInfo.bodyLocY = bodyLoc.y;
		locationInfo.boydLocX = bodyLoc.x;
		//writeForceInformation(img, locationInfo.matchLoc, 0, 0, locationInfo.probeLocX, locationInfo.probeLocY, locationInfo.boydLocX, locationInfo.bodyLocY);

		//update the body center
		bodyCenter = bodyLoc;
		probeAndRobotCenterDistiance = (int)(img.rows / 2) - locationInfo.probeLocY;
	}

	struct LocationInfo matchingMethodProbe(int, void*, Mat img, Mat templ, Mat result, int match_method)
	{
		/// Source image to display
		Point blueProbe = colorTrackingProbeBlue(img);
		cout << "Probe blue dot" << blueProbe << endl;
		struct LocationInfo locationInfo;
		locationInfo.probeLocY = blueProbe.y;
		locationInfo.probeLocX = blueProbe.x;
		locationInfo.probeRows = templ.rows;
		locationInfo.probeCols = templ.cols;
		locationInfo.matchLoc.x = blueProbe.x;
		locationInfo.matchLoc.y = blueProbe.y;


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


	void writeForceInformation(Mat& img_display, int frameToPixel, int probeRows, int probeCols, int probeLocX, int probeLocY, int bodyLocX, int bodyLocY)
	{

		//if (testCounter > 5)
		//{
		//	cv::Point probeLocCorrect = probeStablizer(probeLocX, probeLocY, bodyLocX, bodyLocY);
		//	probeLocX = probeLocCorrect.x;
		//	probeLocY = probeLocCorrect.y;
		//	
		//	//matchLoc.x = probeLocY - probeRows / 2;
		//	//matchLoc.y = probeLocX - probeCols / 2;
		//}

		//Testing show the location of the probe 
		circle(img_display, Point2f(probeLocX, probeLocY), 32.0, cv::Scalar(255, 0, 255), 1, 8);
		circle(img_display, Point2f(bodyLocX, bodyLocY), 32.0, cv::Scalar(255, 0, 255), 1, 8);
		//


		struct Force2D force2D = ForceCalculator(bodyLocX, bodyLocY, probeLocX, probeLocY, frameToPixel);


		int offset = 20;
		///result image
		cv::Scalar info_color = cv::Scalar(0, 255, 0);
		if (abs(force2D.forceY) > 3)
		{
			info_color = cv::Scalar(0, 0, 255);
		}

		std::stringstream ss1;
		ss1 << "Orientation[Degree]: " << orientationAngle << " BodyX: " << roiOffset.x << " BodyY: " << roiOffset.y;
		putText(img_matches, ss1.str(), Point(3, img_matches.rows / 8 * 7.1 - offset), cv::FONT_HERSHEY_TRIPLEX, 1.3, info_color);

		cv::Point probeLocation = findProbeLocation(orientationAngle, roiOffset);
		std::stringstream ss2;
		ss2 << "ProbeX Loc: " << probeLocation.x << " ProbeY Loc: " << probeLocation.y << " Frame Num: " << testCounter;
		putText(img_matches, ss2.str(), Point(3, img_matches.rows / 8 * 7.4 - offset), cv::FONT_HERSHEY_TRIPLEX, 1.3, info_color);

		std::stringstream ss3;
		ss3 << "Y Component Force[microN]: " << std::setprecision(3)<< abs(force2D.forceY);
		putText(img_matches, ss3.str(), Point(3, img_matches.rows / 8 * 7.7 - offset), cv::FONT_HERSHEY_TRIPLEX, 1.3, info_color);

		std::stringstream ss4;

		ss4 << "X Component Force[microN]: " << std::setprecision(3) << force2D.forceX;
		putText(img_matches, ss4.str(), Point(3, img_matches.rows / 8 * 8 - offset + 5), cv::FONT_HERSHEY_TRIPLEX, 1.3, info_color);

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
			//control->update(MicroObject(Point(roiOffset.x, roiOffset.y), robotWidthInPixel, orientationAngle, force2D.forceY));
		}
		else if (testCounter == 5) //set the target info
		{
			control->setTarget(targetLocation, targetDirection);
		}
		//imshow("Template Matching", img_display);
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
		int overall_distance = norm(Point(probeLocX, probeLocY) - Point(bodyLocX, bodyLocY));

		int distanceY = overall_distance * cos(48.1 * PI / 180);
		int distanceX = overall_distance * sin(48.1 * PI / 180);
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
			//double slopeX = 0.059; //this is the slope of the curve fit line that converts distance to force
			//double slopeY = 0.047;
			double bodyLength = 719; //the value for the new robot
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
			if (force2D.forceY < 0.3)	//this is added to eliminate the noise
			{
				force2D.forceY = 0;
			}
			//force2D.forceY = forceXStack(force2D.forceY);

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
		return 0.047 * deformationDistance;
	}

	double slopeXFunction(double deformationDistance)
	{
		return 0.059 * deformationDistance - 0.6227;
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
			//Testing
			//std::cout << "a: " << a.x << " " << a.y << std::endl;
			//std::cout << "b: " << b.x << " " << b.y << std::endl;
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
		if ((bodyLocY - probeLocY) < (-1 * calibrationDistanceY / 3)) // if it is smaller than 50% of the calibration distance, the tracking method is wrong
		{
			probeLocY = bodyLocY + calibrationDistanceY;
		}

		if (abs(bodyLocX - probeLocX) > 20)
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


		int averageForce = (forceXHist[0] + forceXHist[1] + forceXHist[2]) / 3;
		forceXHist[0] = forceXHist[1];
		forceXHist[1] = forceXHist[2];
		forceXHist[2] = forceX;
		
		if (averageForce > 3)
			return forceX;
		else
			return 0;


		return forceX;

	}

	//Colored Tracking Position
	//This is an experimental function to test out the functionality of the color tracking.
	//This function is used to find the green dot on the body
	struct TwoCntrs roi_colorTrackingBody(cv::Mat imageOriginal)
	{
		Point cntr_body;
		Point cntr_spring;

		if (testCounter > 5)
		{
			double lengthOfRectangle = robotWidthInPixel * 4;
			double widthOfRectangle = lengthOfRectangle / 4 * 3;
			Point upperCorner = Point(midPointHist.x - lengthOfRectangle / 2, midPointHist.y - lengthOfRectangle / 2);
			boundCheckUpdate_helper(imageOriginal, upperCorner, lengthOfRectangle, widthOfRectangle);
			cv::Rect Rec(upperCorner.x, upperCorner.y, lengthOfRectangle, widthOfRectangle);
			Mat roi = imageOriginal(Rec);
			Point roi_cntr_body = colorTrackingBody(roi);
			Point roi_cntr_spring = colorTrackingSpring(roi);
			cntr_body = roi_cntr_body + upperCorner;
			cntr_spring = roi_cntr_spring + upperCorner;
		}
		else
		{
			cntr_body = colorTrackingBody(imageOriginal);
			cntr_spring = colorTrackingSpring(imageOriginal);
		}
		struct TwoCntrs twoCntrs;
		twoCntrs.springCntr = cntr_spring;
		twoCntrs.bodyCntr = cntr_body;

		return twoCntrs;
	}

	void boundCheckUpdate_helper(cv::Mat imageOriginal, Point& upperCorner, double& lengthOfRectangle, double& widthOfRectangle)
	{
		if (upperCorner.x < 0)
		{
			lengthOfRectangle += upperCorner.x - 1;
			upperCorner.x = 1;
		}
		if (upperCorner.y < 0)
		{
			widthOfRectangle += upperCorner.y - 1;
			upperCorner.y = 1;
		}

		if (upperCorner.x + widthOfRectangle >= imageOriginal.cols)
		{
			widthOfRectangle = imageOriginal.cols - upperCorner.x - 20;
		}

		if (upperCorner.y + lengthOfRectangle >= imageOriginal.rows)
		{
			widthOfRectangle = imageOriginal.rows - upperCorner.y - 20;
		}

	}

	Point colorTrackingProbeBlue(cv::Mat imageOriginal)
	{
		Mat imgHSV;
		cvtColor(imageOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded;
		inRange(imgHSV, cv::Scalar(37, 0, 154), cv::Scalar(115, 119, 255), imgThresholded); //Threshold the image
																							//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		threshold(imgThresholded, imgThresholded, 75, 230, CV_THRESH_BINARY);

		// Find all the contours in the thresholded image
		vector<Point> contours;
		cv::findNonZero(imgThresholded, contours);
		// Calculate the area of each contour
		int x_mean = 0;
		int y_mean = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			x_mean += contours[i].x;
			y_mean += contours[i].y;
		}
		//no center can be found
		if (contours.size() == 0)
		{
			return Point(-1, -1);
		}
		x_mean = x_mean / contours.size();
		y_mean = y_mean / contours.size();
		Point cntr = Point(x_mean, y_mean);
		//Display the result
		//circle(imgThresholded, cntr, 3, cv::Scalar(255, 0, 255), 2);
		//imshow("Thresholded Image Probe", imgThresholded); //show the thresholded 
		//cv::waitKey(3);
		return cntr;
	}
	Point colorTrackingBody(cv::Mat imageOriginal)
	{
		Mat imgHSV;
		cvtColor(imageOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded;

		inRange(imgHSV, cv::Scalar(65, 66, 45), cv::Scalar(84, 150, 255), imgThresholded); //Threshold the image
																						   //morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		threshold(imgThresholded, imgThresholded, 75, 230, CV_THRESH_BINARY);

		// Find all the contours in the thresholded image
		vector<Point> contours;
		cv::findNonZero(imgThresholded, contours);
		// Calculate the area of each contour
		int x_mean = 0;
		int y_mean = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			x_mean += contours[i].x;
			y_mean += contours[i].y;
		}
		//no center can be found
		if (contours.size() == 0)
		{
			return Point(-1, -1);
		}
		x_mean = x_mean / contours.size();
		y_mean = y_mean / contours.size();
		Point cntr = Point(x_mean, y_mean);

		//Display the result
		//circle(imgThresholded, cntr, 3, cv::Scalar(255, 0, 255), 2);
		//imshow("Thresholded Image Body", imgThresholded); //show the thresholded 
		//cv::waitKey(3);
		return cntr;
	}

	Point colorTrackingSpring(cv::Mat imageOriginal)
	{
		Mat imgHSV;
		cvtColor(imageOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded;
		inRange(imgHSV, cv::Scalar(136, 68, 42), cv::Scalar(182, 183, 255), imgThresholded); //Threshold the image
																							 //morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));


		threshold(imgThresholded, imgThresholded, 75, 230, CV_THRESH_BINARY);
		// Find all the contours in the thresholded image
		vector<cv::Vec4i> hierarchy;
		vector<Point> contours;
		cv::findNonZero(imgThresholded, contours);
		//check corner case
		if (contours.size() == 0) return Point(-1, -1);

		// Find the orientation of each shape
		Point cntr = getCntr_Orientation(contours, imgThresholded);
		//Display the result
		//imshow("Thresholded Image Spring", imgThresholded); //show the thresholded 
		//cv::waitKey(3);
		return cntr;

	}
	Point getCntr_Orientation(const vector<Point> &pts, Mat &img)
	{
		//Construct a buffer used by the pca analysis
		int sz = static_cast<int>(pts.size());
		Mat data_pts = Mat(sz, 2, CV_64FC1);
		for (int i = 0; i < data_pts.rows; ++i)
		{
			data_pts.at<double>(i, 0) = pts[i].x;
			data_pts.at<double>(i, 1) = pts[i].y;
		}
		//Perform PCA analysis
		cv::PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
		//Store the center of the object
		Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
			static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
		//Store the eigenvalues and eigenvectors
		vector<Point2d> eigen_vecs(2);
		vector<double> eigen_val(2);
		for (int i = 0; i < 2; ++i)
		{
			eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
				pca_analysis.eigenvectors.at<double>(i, 1));
			eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
		}
		// Draw the principal components
		circle(img, cntr, 3, cv::Scalar(255, 0, 255), 2);
		//Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
		//Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
		//drawAxis(img, cntr, p1, cv::Scalar(0, 255, 0), 1);
		//drawAxis(img, cntr, p2, cv::Scalar(255, 255, 0), 5);
		//double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
		return cntr;
	}

	//For PCA part
	void drawAxis(Mat& img, Point p, Point q, cv::Scalar colour, const float scale = 0.2)
	{
		double angle;
		double hypotenuse;
		angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
		hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
		//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
		//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
		// Here we lengthen the arrow by a factor of scale
		q.x = (int)(p.x - scale * hypotenuse * cos(angle));
		q.y = (int)(p.y - scale * hypotenuse * sin(angle));
		line(img, p, q, colour, 1, CV_AA);
		// create the arrow hooks
		p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
		p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
		line(img, p, q, colour, 1, CV_AA);
		p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
		p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
		line(img, p, q, colour, 1, CV_AA);
	}

	//This is used for testing the HSV values
	void colorCalibrationHSV(cv::Mat imageOriginal)
	{
		if (testCounter < 2)
		{
			cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
			iLowH = 0;
			iHighH = 179;

			iLowS = 0;
			iHighS = 255;

			iLowV = 0;
			iHighV = 255;

			//Create trackbars in "Control" window
			cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
			cvCreateTrackbar("HighH", "Control", &iHighH, 255);

			cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
			cvCreateTrackbar("HighS", "Control", &iHighS, 255);

			cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
			cvCreateTrackbar("HighV", "Control", &iHighV, 255);
		}

		Mat imgHSV;
		cvtColor(imageOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded;
		inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
																											  //morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, Size(5, 5)));


		threshold(imgThresholded, imgThresholded, 75, 230, CV_THRESH_BINARY);
		imshow("Thresholded Image", imgThresholded); //show the thresholded 

		cv::waitKey(3); //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	}

	/*
	Function: given a picture and a point, find the hsv value of the image
	*/
	void colorSamplingHSV(cv::Mat imageOriginal, Point samplePoint)
	{
		Mat imgHSV;
		cvtColor(imageOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		cv::Vec3b hsv = imgHSV.at<cv::Vec3b>(samplePoint.x, samplePoint.y);
		int h = hsv.val[0];
		int s = hsv.val[1];
		int v = hsv.val[2];

		std::stringstream ss;
		ss << "H: " << h << " S: " << s << " V: " << v;
		imshow(ss.str(), imageOriginal);
		cv::waitKey();
	}
};



#endif
