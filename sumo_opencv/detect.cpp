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

#define PI	(3.1415926535)

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

#if SHOW_FRAME_COORD_SYS
		arrowedLine(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), Point2f(FRAME_WIDTH / 2 + 100., FRAME_HEIGHT / 2), Scalar(0, 0, 255), 2);
		arrowedLine(imageDetected, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2 + 100.), Scalar(0, 255, 0), 2);
#endif

		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		vector<Vec3d> rvecs, tvecs;

		map<int, Mat> marker;
		map<int, Vec3d> markerTvecs;
		map<int, double> phi;

		// Corner detection parameters
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

			// TODO: REMOVE ORIGIN MARKER RESTRICTION
			if (ERR_OK != getOriginXYZ(markerIds, originIndex, rvecs, tvecs, invCamMatrix, markerCorners, xyzOrigin))
			{
				putText(imageDetected, ERR_STR_NO_ORIGIN, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
			}
			else
			{
				//Get World Coordinates for all marker and save them in map "marker"
				//marker[markerId] = xyzCoordinates of marker 
				getMarkerXYZ(markerIds, imageDetected, originIndex, rvecs, tvecs, invCamMatrix, camMatrix, distCoeffs, markerCorners, xyzOrigin, marker);
				//Get Euler Angles of rotation around z-Axis of each marker
				getEulerAngleFromRotMatrix(rvecs, markerIds, phi, originIndex);

#if PRINT_WOLRD_COORDS

				for (size_t i = 0; i < (markerIds.size()); i++)
				{
					if (i != originIndex)
					{
						std::cout << "Marker ID: " << markerIds[i] << " | X:" << marker[markerIds[i]].at<double>(0, 0) << " Y: " << marker[markerIds[i]].at<double>(1, 0) << " Z: " << marker[markerIds[i]].at<double>(2, 0);
						std::cout << " | Phi: " << phi[markerIds[i]]*180/PI << std::endl;
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