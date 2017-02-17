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
int drawArucoMarker(int id, int size, int borderBits, string ofileName);

//Function to read Camera Parameters from file
int readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

//Function to generate Aruco Codes
int generateAruco();