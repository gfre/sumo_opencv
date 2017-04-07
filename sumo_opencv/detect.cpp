#define _USE_MATH_DEFINES
/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"
#include "opencv2\aruco.hpp"
/* PROJECT INCLUDE */
#include "arucoFunc.h"
#include "detect.h"
#include "fileOutput.h"
#include "reconstruct3d.h"
#include "serial.h"
#include "stitcher.h"
#include "config.h"
/* SYSTEM INCLUDES */
#include <cmath>


/* GLOBAL VARIABLES */
static string startStopMsg = "Current State : IDLE - Press 's' to START Sumos";

int detectMarkers()
{
	int returnCode = ERR_OK;

#if CSV_REMOVE_AT_START
	removeCSVFile();
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
	Mat image1, image2, stitchedImage, imageDetected, H;

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

	//Grab frames continuously if at least one webcam is connected 
	while (inputVideo1.grab() | inputVideo2.grab())
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

		//Initialize Variables
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		vector<Vec3d> rvecs, tvecs;

		map<int, Mat> marker;
		map<int, Vec3d> markerTvecs;
		map<int, double> phi;

		// Corner detection parameters
		const Ptr<aruco::DetectorParameters> &param = aruco::DetectorParameters::create();
		param->doCornerRefinement = CR_ENABLE;
		param->cornerRefinementWinSize = CR_WIN_SIZE;
		param->cornerRefinementMaxIterations = CR_MAX_ITERATIONS;
		param->cornerRefinementMinAccuracy = CR_MIN_ACCURACY;

		//Detect Markers
		aruco::detectMarkers(imageDetected, dictionary, markerCorners, markerIds, param);

		//Only proceed if at least 1 marker was detected
		if (markerIds.size() > 0)
		{
			aruco::drawDetectedMarkers(imageDetected, markerCorners, markerIds);
			aruco::estimatePoseSingleMarkers(markerCorners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs, tvecs);

			//Check if the Origin Marker was detected
			int originIndex = -1;
			Vec3d coordOrigin;
			Mat xyzOrigin = Mat::ones(3, 1, DataType<double>::type);

			if ((ERR_OK != getOriginXYZ(markerIds, originIndex, rvecs, tvecs, invCamMatrix, markerCorners, xyzOrigin)) & USE_REL_COORDS)
			{
				putText(imageDetected, ERR_STR_NO_ORIGIN, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
			}
			else
			{
				for (size_t i = 0; i < markerIds.size(); i++)
				{
					if (i != originIndex)
					{
						Mat rotMatrix;
						Vec3d transl = tvecs[i];
						Rodrigues(rvecs[i], rotMatrix);
						Mat invRotMatrix = rotMatrix.inv();

						//Draw Axis of detected marker
						cv::aruco::drawAxis(imageDetected, camMatrix, distCoeffs, rvecs[i], transl, MARKER_LENGTH * 0.5f);

						//Get World Coordinates for all marker and save them in map "marker"
						//marker[markerId] = xyzCoordinates of marker 
						getMarkerXYZ(markerIds[i], invRotMatrix, markerCorners[i], transl, invCamMatrix, marker, xyzOrigin);
						//Get Euler Angles of rotation around z-Axis of each marker
						getEulerAngleFromRotMatrix(rotMatrix, markerIds[i], phi);
					}

				}

#if PRINT_WOLRD_COORDS

				for (size_t i = 0; i < (markerIds.size()); i++)
				{
					if (i != originIndex)
					{
						std::cout << "Marker ID: " << markerIds[i] << " | X:" << marker[markerIds[i]].at<double>(0, 0) << " Y: " << marker[markerIds[i]].at<double>(1, 0) << " Z: " << marker[markerIds[i]].at<double>(2, 0);
						std::cout << " | Phi: " << phi[markerIds[i]]*180/M_PI << std::endl;
					}
				}
#endif


#if PRINT_COORDS_TO_CSV
				if ((markerIds.size() > 1))
				{
#if CSV_SAVE_ID
					writeCoordsToCSV(marker[0], (*markerIds)[0]);
#else
					writeCoordsToCSV(marker[0], CSV_NO_ID);
#endif
				}
#endif


#if SERIAL_TRANSMIT
				uint16_t message[MAX_MSG_LENGTH];

				composeSerialMessage(message, marker, phi);

				//Send Message to COM Port
				sendSerial(SERIAL_COM_PORT, 255, message, sizeof(message) / sizeof(uint8_t));
#endif
#if PRINT_SERIAL_MSG_TO_CL & SERIAL_TRANSMIT
				for (size_t i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
				{
					//if (!marker[i].empty())
						cout << "ID = " << i << " || x = " << (uint16_t)message[3 * i] << " | y = " << (uint16_t)message[3 * i + 1] << " | phi = " << (uint16_t)message[3 * i + 2] << endl;
				}
#endif
			}
		}
		else
		{
			putText(imageDetected, ERR_STR_NO_MARKER, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
		}


#if SHOW_FRAME_CENTER
		circle(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 5, Scalar(0, 0, 255));
#endif

#if SHOW_FRAME_COORD_SYS
		arrowedLine(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), Point2f(FRAME_WIDTH / 2 + 100., FRAME_HEIGHT / 2), Scalar(0, 0, 255), 2);
		arrowedLine(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2 + 100.), Scalar(0, 255, 0), 2);
#endif

		//End loop by pressing "e"
		char c = waitKey(MS_BETWEEN_FRAMES);
		if (101 == c || 69 == c)
		{
			break;
		}

		//Send start message by pressing "s"
		if (115 == c || 83 == c)
		{
			startStopMsg = "Current State : RUNNING - Press 'i' to STOP Sumos";
			
			uint16_t message = VAR_START;

			//Send Message to COM Port
			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "START" << endl;
		}

		//Send stop message by pressing "i"
		if (105 == c || 73 == c)
		{
			startStopMsg = "Current State : IDLE - Press 's' to START Sumos";
			
			//Send Message to COM Port
			//sendSerial(SERIAL_COM_PORT, 255, message, sizeof(message) / sizeof(uint8_t));
			cout << "STOP" << endl;
		}

		String exitMsg = "Press 'e' to exit";
		putText(imageDetected, startStopMsg, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(imageDetected, exitMsg, Point(20, 70), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);

#if SHOW_FINAL_IMAGE
		//Draw image
		namedWindow("Detected Markers", WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
		imshow("Detected Markers", imageDetected);
#endif


}

	inputVideo1.release();

#if USE_STITCHER
	inputVideo2.release();
#endif
	
	return returnCode;
}