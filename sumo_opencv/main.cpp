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
#include <ctime>
/* PROJECT INCLUDES */
#include "config.h"
#include "aruco_func.h"
#include "serial.h"
#include "reconstruct3d.h"
#include "stitcher.h"

/* NAMESPACES */
using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

#if GENERATE_ARUCO_CODES
	for (int i = 0; i <= 25; i++) {
		string fileName, fileFormat, file;

		/* Calculate Pixel --> cm
			https://www.blitzrechner.de/pixel-zentimeter-umrechnen/
			@300dpi - 118px/cm
		*/
		int sizeInPixel = 1772;	// = 15cm @ 300dpi
		int borderBits = 2;

		fileName = to_string(i);
		fileFormat = ".png";
		file = "aruco_codes/" + fileName + fileFormat;
		drawArucoMarker(i, sizeInPixel, borderBits, file);
	}
#endif

	//Camera Parameter xml configuration file
	string configFile = CALIB_FILE_NAME;

	// Open Camera Parameter File
	Mat camMatrix, invCamMatrix, distCoeffs;
	bool readOK = readCameraParameters(configFile, camMatrix, distCoeffs);
	if (!readOK) 
	{
		cerr << "Invalid camera file" << endl;
		return -1;
	}
	invCamMatrix = camMatrix.inv();

#if PRINT_INTR_PARA
	cout << "CamMatrix: " << camMatrix << endl;
	cout << "------------------------------------------" << endl;
	cout << "inv CamMatrix: " << invCamMatrix << endl;
	cout << "============================================" << endl << endl << endl;
#endif

	// Set Aruco Dictionary
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

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

		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		vector<Vec3d> rvecs, tvecs;

		map<int, Mat> marker, cornerBottomLeft;
		map<int, Vec3d> markerTvecs;

		//Detect Markers
		const Ptr<aruco::DetectorParameters> &param = aruco::DetectorParameters::create();
		param->doCornerRefinement = true;
		param->cornerRefinementWinSize = 2;
		param->cornerRefinementMaxIterations = 300;
		param->cornerRefinementMinAccuracy = 0.01;
		aruco::detectMarkers(imageDetected, dictionary, markerCorners, markerIds, param);

		//If at least one marker is detected ...
		if (markerIds.size() > 0)
		{
			aruco::drawDetectedMarkers(imageDetected, markerCorners, markerIds);
			aruco::estimatePoseSingleMarkers(markerCorners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs, tvecs);
			// TODO : Improve Corner detection
			// http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp#cornersubpix
			// http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html

			//Check if the Origin Marker was detected
			int isOriginVisible = 0;
			int originIndex = 0;

			Vec3d coordOrigin;
			Point2f uvOrigin;
			Mat xyzOrigin = Mat::ones(3, 1, DataType<double>::type);

			for (int i = 0; i < markerIds.size(); i++) 
			{
				if (markerIds[i] == ORIGIN_MARKER_ID)
				{
					originIndex = i;
					isOriginVisible = 1;
					coordOrigin = tvecs[originIndex];

					//get Rotation matrix from Rotation vector rvec
					Rodrigues(rvecs[originIndex], rotMatrix);

					//Invert Rotation Matrix once to reduce computational load
					invRotMatrix = rotMatrix.inv();

#if SHIFT_POINT_TO_CENTER
					//Calculate Center of Origin using moments
					Moments mu = moments(markerCorners[i]);
					uvOrigin = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
#else
					//Use top left corner
					uvOrigin.x = markerCorners[i][0].x;
					uvOrigin.y = markerCorners[i][0].y;
#endif

#if PRINT_UV_COORDS 
					cout << "Origin Marker (ID: " << markerIds[i] << ")" << endl;
					cout << "------------------------------------------" << endl;
					cout << "U: " << uvOrigin.x << " V: " << uvOrigin.y << endl;
					cout << "RotMatrix: " << rotMatrix << endl;
					cout << "Transl: " << tvecs[i] << endl;
					cout << "============================================" << endl << endl << endl;
#endif
					//getWorldCoordinates(uvOrigin, &xyzOrigin, invCamMatrix, invRotMatrix, coordOrigin, Z_CONST);

#if PRINT_WOLRD_COORDS 

					cout << "Marker ID: " << markerIds[i] << " - X:" << xyzOrigin.at<double>(0, 0) << " Y: " << xyzOrigin.at<double>(1, 0) << " Z: " << xyzOrigin.at<double>(2, 0) << endl;
#endif
				}
			}

			if (!isOriginVisible)
			{
				const string originNotVisible = "Origin Marker not detected!";
				putText(imageDetected, originNotVisible, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
			}
			else
			{
				//Save all detected markers in map "marker"
				for (unsigned int i = 0; i < markerIds.size(); i++)
				{
					if (i == originIndex) {
						aruco::drawAxis(imageDetected, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 0.5f);
					}
					else
					{
						aruco::drawAxis(imageDetected, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 0.5f);
						//get Rotation matrix from Rotation vector rvec
						Rodrigues(rvecs[i], rotMatrix);
						//Invert Rotation Matrix once to reduce computational load
						invRotMatrix = rotMatrix.inv();

						circle(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 5, Scalar(0, 0, 255));

						Mat xyzPoint = Mat::ones(3, 1, DataType<double>::type), relXYZPoint, xyzBottomLeft = Mat::ones(3, 1, DataType<double>::type);
						Point2f uvPoint, uvBottomLeft;

						//Get Real World Coordinates and copy them to marker map

#if SHIFT_POINT_TO_CENTER
						//Calculate Center using moments
						Moments mu = moments(markerCorners[i]);
						uvPoint = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
						circle(imageDetected, uvPoint, 5, Scalar(0, 255,0));
#else
						//Use top left corner
						uvPoint.x = markerCorners[i][0].x;
						uvPoint.y = markerCorners[i][0].y;
#endif

						//Get Real World Coordinates of Bottom Left Corner for calculation of Phi
						uvBottomLeft.x = markerCorners[i][3].x;
						uvBottomLeft.y = markerCorners[i][3].y;

						//get Rotation matrix from Rotation vector rvec
						Rodrigues(rvecs[i], rotMatrix);
						//Invert Rotation Matrix once to reduce computational load
						invRotMatrix = rotMatrix.inv();

						Mat uv = Mat::ones(2, 1, DataType<double>::type);
						uv.at<double>(0, 0) = uvPoint.x;
						uv.at<double>(1, 0) = uvPoint.y;

#if PRINT_UV_COORDS 
						cout << "ID: " << markerIds[i] << endl;
						cout << "------------------------------------------" << endl;
						cout << "U: " << uvPoint.x << " V: " << uvPoint.y << endl;
						cout << "RotMatrix: " << rotMatrix << endl;
						cout << "Transl: " << tvecs[i] << endl;
 
						cout << "============================================" << endl << endl << endl;
#endif


						getWorldCoordinates(uvPoint, &xyzPoint, invCamMatrix, invRotMatrix, tvecs[i], Z_CONST);
						//getWorldCoordinates(uvBottomLeft, &xyzBottomLeft, invCamMatrix, invRotMatrix, tvecs[i], Z_CONST);

						//cout <<  norm( xyzOrigin, xyzPoint, NORM_L2) << endl;
#if USE_REL_COORDS
						//Get Relative Coordinates (with Coordinate Origin at OriginMarker Top Left Corner)
						xyzPoint.at<double>(0, 0) -= xyzOrigin.at<double>(0, 0);	// rel. x-Coordinate
						xyzPoint.at<double>(1, 0) -= xyzOrigin.at<double>(1, 0);	// rel. y-Coordinate

						xyzBottomLeft.at<double>(0, 0) -= xyzPoint.at<double>(0, 0);	// rel. x-Coordinate
						xyzBottomLeft.at<double>(1, 0) -= xyzPoint.at<double>(1, 0);	// rel. y-Coordinate
#endif
						marker[markerIds[i]] = xyzPoint;
						cornerBottomLeft[markerIds[i]] = xyzBottomLeft;

#if PRINT_WOLRD_COORDS 

						cout << "Marker ID: " << markerIds[i] << " - X:" << marker[markerIds[i]].at<double>(0, 0) << " Y: " << marker[markerIds[i]].at<double>(1, 0) << " Z: " << marker[markerIds[i]].at<double>(2, 0) << endl;
#endif

					}
				}



#if SERIAL_TRANSMIT
				uint16_t message[MAX_MSG_LENGTH];

				for (int i = 0; i < MAX_NUMBER_OF_MARKERS; i++) 
				{
					if (!marker[i].empty()) 
					{
						//Marker with ID = i was detected

						uint16_t phi = (uint16_t)(marker[i].at<double>(0, 0) + cornerBottomLeft[i].at<double>(0, 0));

						message[3 * i] = (uint16_t)(marker[i].at<double>(0, 0)); // x - value
						message[3 * i + 1] = (uint16_t)(marker[i].at<double>(1, 0)); // y - value
						message[3 * i + 2] = phi; // phi - value
					}
					else 
					{
						//Marker with ID = i was NOT detected -> Transmit VAR_INVALID

						message[3 * i] = VAR_INVALID; // x - value
						message[3 * i + 1] = VAR_INVALID; // y - value
						message[3 * i + 2] = VAR_INVALID; // phi; // phi - value
					}
				}

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

}