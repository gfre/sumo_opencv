#define _USE_MATH_DEFINES
/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"
#include "opencv2\aruco.hpp"
/* PROJECT INCLUDE */
#include "arucoFunc.h"
#include "detect.h"
#include "fileOutput.h"
#include "serial.h"
#include "stitcher.h"
#include "config.h"
/* SYSTEM INCLUDES */
#include <cmath>
#include <time.h>


/* GLOBAL VARIABLES */
String currentMsg            = "IDLE";
String recMsg                = "";
static sendState_t sendState = STATE_IDLE;
static recState_t recState   = NO_REC;
static clock_t elapsedTime   = 0;
static clock_t startTime     = 0;
static bool undistortImage   = true;
static uint16_t message[MAX_MSG_LENGTH];

int detectMarkers()
{
	//Camera Parameter xml configuration file
	string configFile = CALIB_FILE_NAME;
	ofstream outputFile;
	cv::Mat camMatrix, invCamMatrix, distCoeffs;
	int returnCode = ERR_OK;
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(ARUCO_DICT);
	VideoCapture inputVideo;

#if CSV_REMOVE_AT_START
	removeCSVFile();
#endif

	outputFile.open(CSV_OUTPUT_FILENAME, ios::out | ios::app);

	// Open Camera Parameter File
	if (ERR_OK != readCameraParameters(configFile, camMatrix, distCoeffs))
	{
		cerr << "Invalid camera file" << endl;
		returnCode = ERR_INV_PARAM_FILE;
	}
	invCamMatrix = camMatrix.inv();

#if PRINT_INTR_PARA
	cout << "Camera Matrix: " << camMatrix << endl;
	cout << "------------------------------------------" << endl;
	cout << "Inv Camera Matrix: " << invCamMatrix << endl;
	cout << "------------------------------------------" << endl;
	cout << "Distortion coefficients: " << distCoeffs << endl;
	cout << "============================================" << endl << endl << endl;
#endif



	//Select Webcams
	inputVideo.open(0);

	//Set Resolution
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo.set(CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH);

	//initialize at objects
	Mat image;

#if ENABLE_REC
	//Initialize Videowriter
	VideoWriter vidWriter;

	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	double fps = (double)REC_FPS;
	string vidFilename = "rec.avi";
	Mat vidSizeImg;

	//Save on image to get size
	inputVideo >> vidSizeImg;

	bool isColor = (vidSizeImg.type() == CV_8UC3);

	vidWriter.open(vidFilename, codec, fps, vidSizeImg.size(), isColor);

	if (!vidWriter.isOpened()) {
		cerr << "Could not open the output video file for write\n";
	}

#endif

	//Grab frames continuously if at least one webcam is connected
	while (inputVideo.grab())
	{
		inputVideo.retrieve(image);
#if ENABLE_REC
		if (recState == REC)
		{
			vidWriter.write(image);
		}
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
		param->cornerRefinementWinSize = CR_WIN_SIZE;
		param->cornerRefinementMaxIterations = CR_MAX_ITERATIONS;
		param->cornerRefinementMinAccuracy = CR_MIN_ACCURACY;

		//Detect Marker corners
		aruco::detectMarkers(image, dictionary, markerCorners, markerIds, param);

		//Only proceed if at least 1 marker was detected
		if (markerIds.size() > 0)
		{
			aruco::drawDetectedMarkers(image, markerCorners, markerIds);
			// Estimate poses from image
			aruco::estimatePoseSingleMarkers(markerCorners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs, tvecs);	

				for (size_t i = 0; i < markerIds.size(); i++)
				{
						cv::Mat t, rotMatrix, invRotMatrix;

						// Compose rotation matrix from rotation vector
						Rodrigues(rvecs[i], rotMatrix);
						// Inverse is its transpose
						invRotMatrix = rotMatrix.t();
#if PRINT_ROT_MATRIX
						cout << "Rot Matrix: " << rotMatrix << endl;
#endif

						//Draw Axis of detected marker
						cv::aruco::drawAxis(image, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 0.5f);

						//Copy translation vector tvec to cv::Mat object t
						t = cv::Mat::ones(3, 1, cv::DataType<double>::type);
						t.at<double>(0, 0) = tvecs[i][0];
						t.at<double>(1, 0) = tvecs[i][1];
						t.at<double>(2, 0) = tvecs[i][2];

						phi[markerIds[i]]    = atan2(rotMatrix.at<double>(0, 1), -rotMatrix.at<double>(0, 0));
						marker[markerIds[i]] = t;
				}

#if PRINT_WORLD_COORDS
				for (size_t i = 0; i < (markerIds.size()); i++)
				{
						std::cout << "Marker ID: " << markerIds[i] << " | X:" << marker[markerIds[i]].at<double>(0, 0) << " Y: " << marker[markerIds[i]].at<double>(1, 0) << " Z: " << marker[markerIds[i]].at<double>(2, 0);
						std::cout << " | Phi: " << phi[markerIds[i]] * 180 / M_PI << std::endl;
				}
#endif


#if SERIAL_TRANSMIT
				composeSerialMessage(message, marker, phi);
				sendSerial(SERIAL_COM_PORT, 255, message, sizeof(message) / sizeof(uint8_t));

#endif
#if PRINT_SERIAL_MSG_TO_CL & SERIAL_TRANSMIT
				for (size_t i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
				{
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
		else
		{
			putText(image, ERR_STR_NO_MARKER, Point(500, 520), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 255), 2);
		}
#if SHOW_FRAME_CENTER
		circle(image, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 5, Scalar(0, 0, 255));
#endif
#if SHOW_FRAME_COORD_SYS
		arrowedLine(image, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), Point2f(FRAME_WIDTH / 2 - 100., FRAME_HEIGHT / 2), Scalar(0, 0, 255), 2);
		arrowedLine(image, Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), Point2f(FRAME_WIDTH / 2, FRAME_HEIGHT / 2 + 100.), Scalar(0, 255, 0), 2);
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

			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "READY" << endl;
		}
		//Send stop message by pressing "i"
		if (105 == c || 73 == c)
		{
			sendState = STATE_IDLE;
			uint16_t message = VAR_STOP;
			currentMsg = "IDLE";

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

			sendSerial(SERIAL_COM_PORT, 255, &message, sizeof(message) / sizeof(uint8_t));
			cout << "Start Next Transition" << endl;
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
		String readyMsg     = "Press 'r' to enter READY";
		String stopMsg      = "Press 'i' to reset to IDLE";
		String transMsg     = "Press 't' to start next Transition";
		String exitMsg      = "Press 'e' to EXIT Program";
		putText(image, readyMsg,				  Point(20, 30),   FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(image, stopMsg,					  Point(20, 70),   FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(image, transMsg,			      Point(20, 110),  FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(image, exitMsg,					  Point(20, 190),  FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(image, String("Current State: "), Point(400, 90),  FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
		putText(image, currentMsg,				  Point(600, 90),  FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
		putText(image, recMsg,					  Point(600, 150), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);

#if SHOW_FINAL_IMAGE
		//Set Window
		namedWindow("Detected Markers", WINDOW_NORMAL | CV_GUI_EXPANDED);
		//Draw image
		imshow("Detected Markers", image);
#endif
	}
	inputVideo.release();
	return returnCode;
}