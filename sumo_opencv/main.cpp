/*
Author : Nico Engel, nice@tf.uni-kiel.de, CAU-Kiel
Date   : 17.02.2017
*/

/* PROJECT INCLUDES */
#include "config.h"
#include "arucoFunc.h"
#include "detect.h"
#include <boost/thread/thread.hpp>

/* NAMESPACES */
using namespace std;
using namespace cv;

class ContinuousCapture 
{
	VideoCapture inputVideo_;
public:
	cv::Mat image_;
	bool videoIsOpened_ = false;

	ContinuousCapture() {
		videoIsOpened_ = inputVideo_.open(0);
	}
	ContinuousCapture(int cameraIndex) {
		videoIsOpened_ = inputVideo_.open(cameraIndex);
	}
	ContinuousCapture(double FrameWidth, double FrameHeight) {
		videoIsOpened_ = inputVideo_.open(0);
		inputVideo_.set(CAP_PROP_FRAME_HEIGHT, FrameHeight);
		inputVideo_.set(CAP_PROP_FRAME_WIDTH, FrameWidth);
	}
	ContinuousCapture(int cameraIndex, double FrameWidth, double FrameHeight) {
		videoIsOpened_ = inputVideo_.open(cameraIndex);
		inputVideo_.set(CAP_PROP_FRAME_HEIGHT, FrameHeight);
		inputVideo_.set(CAP_PROP_FRAME_WIDTH, FrameWidth);
	}
	void capture()
	{
		if (videoIsOpened_)
		{
			while (inputVideo_.read(image_))
			{

			}
		}
	}
};

class ArucoDetection 
{
	Ptr<aruco::Dictionary> dictionary;
public:
	
};

ContinuousCapture VidCapture(0, 2592, 2048);


void detectMarkers2()
{

}

int main(int argc, char *argv[])
{
	boost::thread CaptureThread(VidCapture.capture);
	boost::thread DetectMarkersThread();
	int error = ERR_OK;
#if GENERATE_ARUCO_CODES == SUMO_OPENCV_MODE
	error = generateAruco();
#elif CALIBRATION_CAPTURE_SAVE_IMAGES == SUMO_OPENCV_MODE
	error = captureSaveCalibImages();
#elif CALIBRATION_CALIBRATE_CAMERA == SUMO_OPENCV_MODE
	error = calibrateCamera();
#else
	error = detectMarkers();
#endif

	return error;

}

