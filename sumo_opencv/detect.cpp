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
#include <time.h>


/* GLOBAL VARIABLES */
String currentMsg = "IDLE";
String recMsg = "";
static sendState_t sendState = STATE_IDLE;
static recState_t recState = NO_REC;
static clock_t elapsedTime = 0;
static clock_t startTime = 0;
static bool undistortImage = true;

static uint16_t message[MAX_MSG_LENGTH];

int detectMarkers()
{
	int returnCode = ERR_OK;
#if CSV_REMOVE_AT_START
	removeCSVFile();
#endif

	//Camera Parameter xml configuration file
	string configFile = CALIB_FILE_NAME;
	ofstream outputFile;
	outputFile.open(CSV_OUTPUT_FILENAME, ios::out | ios::app);

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
	getHomographyMatrix(image1, image2, &H, MIN_HESSIAN, 100.0, true);
	// Print new Homography Matrix for saving
	cout << H;
#else
	// Use "old" Homography Matrix
	H = (Mat_<double>(3, 3) << HOMOGRAPHY_M);
#endif



#if ENABLE_REC
	//Initialize Videowriter
	VideoWriter vidWriter;

	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	double fps = (double)REC_FPS;
	string vidFilename = "rec.avi";
	Mat vidSizeImg;

#if USE_STITCHER

	inputVideo1.retrieve(image1);
	inputVideo2.retrieve(image2);

	stitcher(image1, image2, &stitchedImage, H);

	//Crop stitched image
	Rect cropROI(0, 0, 1900, 1900);
	vidSizeImg = stitchedImage(cropROI);

#else
	//Save on image to get size
	inputVideo1 >> vidSizeImg;
#endif

	bool isColor = (vidSizeImg.type() == CV_8UC3);

	vidWriter.open(vidFilename, codec, fps, vidSizeImg.size(), isColor);

	if (!vidWriter.isOpened()) {
		cerr << "Could not open the output video file for write\n";
	}

#endif



	//Grab frames continuously if at least one webcam is connected
	while (inputVideo1.grab() | inputVideo2.grab())
	{
		inputVideo1.retrieve(image1);



#if USE_STITCHER
		inputVideo2.retrieve(image2);

		stitcher(image1, image2, &stitchedImage, H);

		//Crop stitched image
		Rect cropROI(0, 0, 1900, 1900);
		imageDetected = stitchedImage(cropROI);

#if ENABLE_REC 
		if (recState == REC)
		{
			vidWriter.write(imageDetected);
		}
#endif


#else
		image1.copyTo(imageDetected);

#if ENABLE_REC
		if (recState == REC)
		{
			vidWriter.write(image1);
		}
#endif

#endif
		
		//Undistort Image if needed
		//Mat imageUndistorted;
		//imageDetected.copyTo(imageUndistorted);
		//if (undistortImage) {
		//	undistort(imageDetected, imageUndistorted, camMatrix, distCoeffs);
		//}
		
		//Initialize Variables
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		vector<Vec3d> rvecs, tvecs;

		map<int, Mat> marker;
		map<int, Vec3d> markerTvecs;
		map<int, double> phi;

		// Corner detection parameters
		const Ptr<aruco::DetectorParameters> &param = aruco::DetectorParameters::create();
		//param->doCornerRefinement = CR_ENABLE;
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
				putText(imageDetected, ERR_STR_NO_ORIGIN, Point(800, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
			}
			else
			{
				for (size_t i = 0; i < markerIds.size(); i++)
				{
					if (i != originIndex)
					{
						cv::Mat XYZ, xyz, t, rotMatrix;
						//Vec3d transl = tvecs[i];
						Rodrigues(rvecs[i], rotMatrix);
						//Mat invRotMatrix = rotMatrix.inv();

						//Draw Axis of detected marker
						cv::aruco::drawAxis(imageDetected, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 0.5f);

#if SHIFT_POINT_TO_CENTER
						//Calculate Center using moments
						cv::Moments mu = cv::moments(markerCorners[i]);
						XYZ = cv::Mat::ones(3, 1, cv::DataType<double>::type);
						XYZ.at<double>(0, 0) = mu.m10 / mu.m00;
						XYZ.at<double>(1, 0) = mu.m01 / mu.m00;
						XYZ.at<double>(2, 0) = Z_CONST;
						std::cout << "XYZ(1): " << XYZ.at<double>(0, 0) << std::endl;
						std::cout << "XYZ(2): " << XYZ.at<double>(1, 0) << std::endl;
						std::cout << "XYZ(3): " << XYZ.at<double>(2, 0) << std::endl;

#else
						//Use top left corner
						XYZ.at<double>(0, 0) = markerCorners[0].x;
						XYZ.at<double>(1, 0) = markerCorners[0].y;
						XYZ.at<double>(2, 0) = Z_CONST;
#endif
						//Copy translation vector tvec to cv::Mat object t
						t = cv::Mat::ones(3, 1, cv::DataType<double>::type);
						t.at<double>(0, 0) = tvecs[i][0];
						t.at<double>(1, 0) = tvecs[i][1];
						t.at<double>(2, 0) = tvecs[i][2];

						std::cout << "t(1): " << t.at<double>(0, 0) << std::endl;
						std::cout << "t(2): " << t.at<double>(1, 0) << std::endl;
						std::cout << "t(2): " << t.at<double>(2, 0) << std::endl;
						//Get World Coordinates for all marker and save them in map "marker"
						//marker[markerId] = xyzCoordinates of marker
						//getMarkerXYZ(markerIds[i], invRotMatrix, markerCorners[i], tvecs[i], invCamMatrix, marker, xyzOrigin);
						//Get Euler Angles of rotation around z-Axis of each marker
						getEulerAngleFromRotMatrix(rotMatrix, markerIds[i], phi);
						marker[markerIds[i]] = xyz;

					}
				}

#if PRINT_WOLRD_COORDS

				for (size_t i = 0; i < (markerIds.size()); i++)
				{
					if (i != originIndex)
					{
						std::cout << "Marker ID: " << markerIds[i] << " | X:" << marker[markerIds[i]].at<double>(0, 0) << " Y: " << marker[markerIds[i]].at<double>(1, 0) << " Z: " << marker[markerIds[i]].at<double>(2, 0);
						std::cout << " | Phi: " << phi[markerIds[i]] * 180 / M_PI << std::endl;
					}
				}


#endif


#if SERIAL_TRANSMIT

				composeSerialMessage(message, marker, phi);

				//Send Message to COM Port
				sendSerial(SERIAL_COM_PORT, 255, message, sizeof(message) / sizeof(uint8_t));

#endif
#if PRINT_SERIAL_MSG_TO_CL & SERIAL_TRANSMIT
				for (size_t i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
				{
					//if (!marker[i].empty())
					cout << "ID = " << i << " || x = " << (int16_t)message[3 * i] << " | y = " << (int16_t)message[3 * i + 1] << " | phi = " << (int16_t)message[3 * i + 2] << endl;
				}
#endif
				

#if PRINT_COORDS_TO_CSV
				long double timestamp = 0.0;

				if ((markerIds.size() > 1))
				{
#if CSV_SAVE_ID		

					if (recState == REC)
					{
						if (startTime == 0)
						{
							startTime = clock();
						}

						elapsedTime = clock() - startTime;
						timestamp = (long double)elapsedTime / (long double)(CLOCKS_PER_SEC * 100.0);

						for (size_t i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
						{
							writeMsgToCSV((int16_t *)message, MAX_MSG_LENGTH, timestamp, &outputFile);

						}

						
					}
#else
					writeCoordsToCSV(marker[0], CSV_NO_ID);
#endif
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

		//Send start message by pressing "r"
		if (114 == c || 82 == c)
		{
			uint16_t message = VAR_READY;
			sendState = STATE_READY;
			currentMsg = "READY";

			//Send Message to COM Port
			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "READY" << endl;
		}
#if 0
		//Send start message by pressing "s"
		if (115 == c || 83 == c)
		{
			uint16_t message = VAR_START;
			sendState = STATE_RUN;
			currentMsg = "START";

			//Send Message to COM Port
			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "START" << endl;
		}

#endif
		//Send stop message by pressing "i"
		if (105 == c || 73 == c)
		{
			sendState = STATE_IDLE;
			uint16_t message = VAR_STOP;
			currentMsg = "IDLE";

			//Send Message to COM Port
			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "STOP" << endl;
		}

		//Next Transition message by pressing "t"
		if (116 == c || 84 == c)
		{
			uint16_t message = VAR_TRANS;

			if (recState == NO_REC && START_REC_WITH_TRANSITION)
			{
				recState = REC;
				recMsg = "RECORDING...";
			}
			//Send Message to COM Port
			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "Start Next Transition" << endl;
		}

		//Toggle undistortion by pressing "g"
		if (103 == c || 71 == c)
		{
			if (undistortImage) {
				undistortImage = false;
			} else {
				undistortImage = true;
			}
		}

#if ENABLE_REC
		//record by pressing "w"
		if (119 == c || 87 == c)
		{
			if (recState == NO_REC)
			{
				recState = REC;
				recMsg = "RECORDING...";
			}
			else
			{
				recState = NO_REC;
				recMsg = "";
				vidWriter.release();
				outputFile.close();
			}
		}
#endif

		/* Show Config Messages on the Screen */
		String readyMsg = "Press 'r' to enter READY";
		//String startMsg = "Press 's' to START";
		String stopMsg = "Press 'i' to reset to IDLE";
		String transMsg = "Press 't' to start next Transition";
		String exitMsg = "Press 'e' to EXIT Program";
		String undistortMsg = "Press 'g' to toggle Distortion";
		putText(imageDetected, readyMsg, Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		//putText(imageUndistorted, startMsg, Point(20, 70), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(imageDetected, stopMsg, Point(20, 70), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(imageDetected, transMsg, Point(20, 110), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);

		putText(imageDetected, undistortMsg, Point(20, 150), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);

		putText(imageDetected, exitMsg, Point(20, 190), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);


		putText(imageDetected, String("Current State: "), Point(400, 90), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(imageDetected, currentMsg, Point(600, 90), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);

		putText(imageDetected, recMsg, Point(600, 150), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);

		//Set Window
		namedWindow("Detected Markers", WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
#if SHOW_FINAL_IMAGE
		//Draw image
		imshow("Detected Markers", imageDetected);
#endif
	}

	inputVideo1.release();

#if USE_STITCHER
	inputVideo2.release();
#endif

	return returnCode;
}