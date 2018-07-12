/*
Author : Nico Engel, nice@tf.uni-kiel.de, CAU-Kiel
Date   : 17.02.2017
*/

/* PROJECT INCLUDES */
#include "config.h"
#include "arucoFunc.h"
#include "detect.h"


/* NAMESPACES */
using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{

#if GENERATE_ARUCO_CODES == SUMO_OPENCV_MODE
	generateAruco();
#elif CALIBRATION_CAPTURE_IMAGES == SUMO_OPENCV_MODE
	captureImages();
#elif CALIBRATION_CALIBRATE_CAMERA == SUMO_OPENCV_MODE
	calibrateCamera();
#else
	detectMarkers();

#endif

	return ERR_OK;

}

