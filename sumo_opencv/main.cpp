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

