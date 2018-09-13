/*
Author : Nico Engel, nice@tf.uni-kiel.de, CAU-Kiel
Date   : 17.02.2017
*/

/* PROJECT INCLUDES */
#include "config.h"
#include "arucoFunc.h"
#include "detect.h"
#include "time.h"
#include <boost/thread/thread.hpp>

/* NAMESPACES */
using namespace std;
using namespace cv;

cv::Mat image;
VideoCapture inputVideo;
clock_t captureClock;


void capture()
{
	inputVideo.open(0);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	while (inputVideo.read(image))
	{
		captureClock = clock();
		//namedWindow("Original Image", WINDOW_NORMAL | CV_GUI_EXPANDED);
		char c = waitKey(1);
		cv::imshow("Original Image", image);
		
		std::cout << "Time elapsed for capture in [ms]: " << ((double)(clock() - captureClock)) << std::endl;
	}
}


int main(int argc, char *argv[])
{
	boost::thread CaptureThread(capture);
	CaptureThread.join();
	//boost::thread DetectMarkersThread(detectMarkers2);


}

