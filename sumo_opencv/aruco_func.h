#pragma once
#include "opencv2\aruco.hpp"
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"

#include <string>

using namespace cv;
using namespace std;

//Function to draw Aruco marker with id, size in pixel, borderBits (must be greater or equal 1), and filename
void drawArucoMarker(int id, int size, int borderBits, string ofileName);

//Function to read Camera Parameters from file
bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);