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
clock_t captureClock, showClock;


void capture()
{
	inputVideo.open(0);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	while (1)
	{
		captureClock = clock();
		inputVideo.read(image);
		std::cout << "Time elapsed for capture in [ms]: " << ((double)(clock() - captureClock)) << std::endl;
	}
}

void show()
{
	for (;;)
	{
		showClock = clock();
		if (!image.empty())
		{
			namedWindow("Original Image", WINDOW_NORMAL | CV_GUI_EXPANDED);
			cv::imshow("Original Image", image);
			waitKey(1);
		}
		std::cout << "Time elapsed for show in [ms]: " << ((double)(clock() - showClock)) << std::endl;
	}
}

int main(int argc, char *argv[])
{
	boost::thread CaptureThread(capture);
	boost::thread ShowThread(show);

	for (;;) {}
	//boost::thread DetectMarkersThread(detectMarkers2);


}

