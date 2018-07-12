#pragma once
/* SYSTEM INCLUDES */
#include <string>
/* PROJECT INCLUDE */
#include "arucoFunc.h"
#include "detect.h"
#include "fileOutput.h"
#include "serial.h"

/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"
#include "opencv2\aruco.hpp"

using namespace cv;
using namespace std;

//Function to draw Aruco marker with id, size in pixel, borderBits (must be greater or equal 1), and filename
int drawArucoMarker(int id, int size, int borderBits, string ofileName);

//Function to read Camera Parameters from file
int readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);

//Function to generate Aruco Codes
int generateAruco();

//Function that captures and saves images for camera calibration
int captureImages();

//Function to calibrate Camera using Charuco Board
int calibrateCamera();

int createCharucoBoard(string filename, int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId, int borderBits);