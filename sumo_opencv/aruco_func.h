#pragma once

/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
/* SYSTEM INCLUDES */
#include <string>
/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;
using namespace std;

//Function to draw Aruco marker with id, size in pixel, borderBits (must be greater or equal 1), and filename
Std_Rtn_Type drawArucoMarker(int id, int size, int borderBits, string ofileName);

//Function to read Camera Parameters from file
Std_Rtn_Type readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);