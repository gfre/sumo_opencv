/*
Author: Nico Engel

*/

/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
#include "opencv2\aruco.hpp"
#include "opencv2\opencv_modules.hpp"
/* SYSTEM INCLUDES */
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <stdint.h>
#include <map>
#include <math.h>
/* PROJECT INCLUDES */
#include "config.h"
#include "aruco_func.h"
#include "serial.h"
#include "reconstruct3d.h"
#include "stitcher.h"

/* NAMESPACES */
using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{

	int returnCode = ERR_OK;

#if GENERATE_ARUCO_CODES
	generateAruco();
#endif

	//Camera Parameter xml configuration file
	string configFile = CALIB_FILE_NAME;

	// Open Camera Parameter File
	cv::Mat camMatrix, invCamMatrix, distCoeffs;
	if (ERR_OK != readCameraParameters(configFile, camMatrix, distCoeffs))
	{
		cerr << "Invalid camera file" << endl;
		returnCode = ERR_INV_PARAM_FILE;
	}
	invCamMatrix = camMatrix.inv();
	cout << distCoeffs << endl;

#if PRINT_INTR_PARA
	cout << "CamMatrix: " << camMatrix << endl;
	cout << "------------------------------------------" << endl;
	cout << "inv CamMatrix: " << invCamMatrix << endl;
	cout << "============================================" << endl << endl << endl;
#endif

	// Set Aruco Dictionary
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(ARUCO_DICT);

	//Open Webcam
	VideoCapture inputVideo1, inputVideo2;

	//Select Webcams
	inputVideo1.open(FIRST_CAM_ID);

	//Set Resolution
	inputVideo1.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo1.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);

#if USE_STITCHER
	inputVideo2.open(SEC_CAM_ID);
	//Set Resolution
	inputVideo2.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo2.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
#endif

	//initialize Mat objects
	Mat image1, image2, stitchedImage, imageDetected, rotMatrix, invRotMatrix, H;

#if RECALCULATE_HOMOGRAPHY
	inputVideo1.retrieve(image1);
	inputVideo2.retrieve(image2);
	getHomographyMatrix(image1, image2, &H, MIN_HESSIAN, 100.0, false);
	// Print new Homography Matrix for saving
	//cout << H;
#else
	// Use "old" Homography Matrix
	H = (Mat_<double>(3, 3) << HOMOGRAPHY_M);
#endif

	//Grab frames continuously
	while (true)
	{
		inputVideo1.retrieve(image1);

#if USE_STITCHER
		inputVideo2.retrieve(image2);

		stitcher(image1, image2, &stitchedImage, H);
		stitchedImage.copyTo(imageDetected);
#else
		image1.copyTo(imageDetected);
#endif

#if UNDISTORT_IMAGE
		Mat imageUndistorted;
		undistort(imageDetected, imageUndistorted, camMatrix, distCoeffs);
#endif

#if SHOW_FRAME_CENTER
		circle(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 5, Scalar(0, 0, 255));
#endif

		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		vector<Vec3d> rvecs, tvecs;

		map<int, Mat> marker, cornerBottomLeft;
		map<int, Vec3d> markerTvecs;

		// TODO : Improve Corner detection parameters
		// http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp#cornersubpix
		// http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html
		const Ptr<aruco::DetectorParameters> &param = aruco::DetectorParameters::create();
		param->doCornerRefinement = true;
		param->cornerRefinementWinSize = 3;
		param->cornerRefinementMaxIterations = 50;
		param->cornerRefinementMinAccuracy = 0.1;

		//Detect Markers
		aruco::detectMarkers(imageDetected, dictionary, markerCorners, markerIds, param);

		//Only proceed if at least 1 marker was detected
		if (markerIds.size() > 0)
		{
			aruco::drawDetectedMarkers(imageDetected, markerCorners, markerIds);
			aruco::estimatePoseSingleMarkers(markerCorners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs, tvecs);

			//Check if the Origin Marker was detected
			int originIndex = 0;

			Vec3d coordOrigin;
			Mat xyzOrigin = Mat::ones(3, 1, DataType<double>::type);


			if (ERR_OK != getOriginXYZ(&markerIds, &originIndex, &rvecs, &tvecs, &invCamMatrix, &markerCorners, &xyzOrigin))
			{
				putText(imageDetected, ERR_STR_NO_ORIGIN, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
			}
			else
			{

				getMarkerXYZ(&markerIds, &imageDetected, originIndex, &rvecs, &tvecs, &invCamMatrix, &camMatrix, &distCoeffs, &markerCorners, &xyzOrigin, &marker, &cornerBottomLeft);

#if SERIAL_TRANSMIT
				uint16 message[MAX_MSG_LENGTH];

				

				//Send Message to COM Port
				sendSerial("COM2", 255, message, sizeof(message) / sizeof(uint8_t));
#endif
#if PRINT_SERIAL_MSG_TO_CL && SERIAL_TRANSMIT
				for (size_t i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
				{
					if (!marker[i].empty())
						cout << "ID = " << i << " || x = " << (int16_t)message[3 * i] << " | y = " << (int16_t)message[3 * i + 1] << " | phi = " << (int16_t)message[3 * i + 2] << endl;
				}
#endif
			}
		}
		else
		{
			putText(imageDetected, ERR_STR_NO_MARKER, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
		}

#if SHOW_FINAL_IMAGE
		//Draw image
		namedWindow("Detected Markers", WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
		imshow("Detected Markers", imageDetected);
#endif

		//End loop by pressing "c"
		char c = waitKey(MS_BETWEEN_FRAMES);
		if (67 == c || 99 == c)
		{
			break;
		}

	}

	inputVideo1.release();

#if USE_STITCHER
	inputVideo2.release();
#endif

	return returnCode;

}