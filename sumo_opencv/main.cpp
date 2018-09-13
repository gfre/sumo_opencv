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
boost::mutex imageMutex;
clock_t captureClock;


void capture()
{
	inputVideo.open(0);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	while (1)
	{
		captureClock = clock();
		namedWindow("Original Image", WINDOW_NORMAL | CV_GUI_EXPANDED);
		imageMutex.lock();
		inputVideo.read(image);
		cv::imshow("Original Image", image);
		imageMutex.unlock();
		waitKey(1);
		std::cout << "Time elapsed for capture in [ms]: " << ((double)(clock() - captureClock)) << std::endl;
	}
}


int main(int argc, char *argv[])
{
	boost::thread CaptureThread(capture);
	for (;;);
}

