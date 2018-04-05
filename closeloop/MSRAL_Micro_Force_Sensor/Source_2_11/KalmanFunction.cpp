#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"


Point KalmanStablizer(Point bodyPos)
{
	if(testCounter < 2)
	{
		KalmanFilter KF(4, 2, 0);	//class variable
		KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
		Mat_<float> measurement(2,1); //class variable
		measurement.setTo(Scalar(0));		


		KF.statePre.at<float>(0) = bodyPos.x;
		KF.statePre.at<float>(1) = bodyPos.y;
		KF.statePre.at<float>(2) = 0;
		KF.statePre.at<float>(3) = 0;


		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
		setIdentity(KF.measurementNoiseCov, Scalar::all(10));
		setIdentity(KF.errorCovPost, Scalar::all(.1));
	}

	// First predict, to update the internal statePre variable
	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
	              
	// Get body Position
	measurement(0) = bodyPos.x;
	measurement(1) = bodyPos.y; 
	  
	// The update phase 
	Mat estimated = KF.correct(measurement);
	Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	return statePt;
}