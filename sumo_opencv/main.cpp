/*
Author: Nico Engel
*/


/* OPENCV INCLUDES */
#include "opencv2\aruco.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\calib3d.hpp"
#include "opencv2\plot.hpp"
#include "opencv2\xfeatures2d.hpp"
#include "opencv2\aruco\charuco.hpp"

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

/* NAMESPACES */
using namespace std;
using namespace cv;

/* SETUP */
#define M_PI					(3.14159265358979323846)					//Pi

#define SERIAL_TRANSMIT			(0)											//Enable/Disable Serial Transmission
#define RECALCULATE_HOMOGRAPHY	(0)											//Set to 1 if camera settings/position changed or Homography Matrix needs to be calculated again

#define NATIVE_WIDTH			(640)										//Do not change
#define NATIVE_HEIGHT			(360)										//Do not change
#define RES_SCALING_FACTOR		(3)											/*Set Video resolution
1 for 640x360
2 for 1280x720
3 for 1920x1080 */
#define MARKER_LENGTH			(100)										//Marker length in mm
#define MAX_NUMBER_OF_MARKERS	(4)											//How many markers/robots exist
#define NUM_OF_VARIABLES		(3)											//How many variables per robot (x, y, phi)
#define MAX_MSG_LENGTH ((MAX_NUMBER_OF_MARKERS)*(NUM_OF_VARIABLES))
#define ORIGIN_MARKER_ID		(24)										//Select which marker acts as the origin of world coordinate system
#define MIN_HESSIAN				(500)										//Minimum Hessian threshold for Surf Algorithm
#define Z_CONST					(3050+940)										//Distance from Camera to Marker Plane in mm

#define HOMOGRAPHY_M			0.9568005531058007, 0.001879261708537507, 488.6830366627206, -0.02376243394348695, 0.9520233532133866, 17.3904366427672, -5.302341910721238e-06, -2.612583953997515e-05, 1

#define ERR_OK					(1)							
#define VAR_INVALID				(0xFFEEu)									//Send this instead of coordinates if marker was not detected




//Function to draw Aruco marker with id, size in pixel, borderBits (must be greater or equal 1), and filename
void drawArucoMarker(int id, int size, int borderBits, string ofileName) {
	/* Draw Aruco Marker
	*/
	Mat markerImage;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	aruco::drawMarker(dictionary, id, size, markerImage, borderBits);

	//namedWindow("Aruco Marker", WINDOW_AUTOSIZE);
	//imshow("Aruco Marker", markerImage);
	imwrite(ofileName, markerImage);
	//waitKey(0);
	cout << ofileName << " Marker succesfully created" << endl;
}

//Function to read Camera Parameters from file
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

//Get World Coordinates (X,Y,Z) from Image Coordinates (u,v,1)
int getWorldCoordinates(Point2f uv, Mat *xyz, Mat invCamMatrix, Mat invRotMatrix, Vec3d tvec, double zConst) {

	double s;
	Mat uvPoint, t, tmp, tmp2;

	uvPoint = Mat::ones(3, 1, DataType<double>::type);
	uvPoint.at<double>(0, 0) = uv.x;
	uvPoint.at<double>(1, 0) = uv.y;

	//Copy translation vector tvec to Mat object t
	t = Mat::ones(3, 1, DataType<double>::type);
	t.at<double>(0, 0) = tvec[0];
	t.at<double>(1, 0) = tvec[1];
	t.at<double>(2, 0) = tvec[2];

	tmp = invRotMatrix * invCamMatrix * uvPoint;
	s = (zConst - t.at<double>(2, 0)) / tmp.at<double>(2, 0);
	*xyz = t + s * tmp;

	/*
	//calculate temp values to solve eq for scaling factor s
	tmp = invRotMatrix * invCamMatrix * uvPoint;
	tmp2 = invRotMatrix * t;

	s = (zConst + tmp2.at<double>(2, 0)) / tmp.at<double>(2, 0);

	*xyz = invRotMatrix * (s * invCamMatrix * uvPoint - t);
	*/
	return ERR_OK;

}

//Send data via serial communication port
int sendSerial(char *serialPort, int maxBufSize, uint16_t *sendBuf, uint8_t byteSize) {
	/*
	https://msdn.microsoft.com/en-us/library/ff802693.aspx
	http://www.dreamincode.net/forums/topic/322031-serial-communication-with-c-programming/
	*/

	/*Serial Setup*/
	HANDLE hComm;
	BOOL hStatus;
	DCB dcb = { 0 };
	COMMTIMEOUTS timeouts = { 0 };
	DWORD dwBytesWrite = 0;

	/*Timeouts*/
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	/*Open Comm Port
	https://msdn.microsoft.com/en-us/library/windows/desktop/aa363858(v=vs.85).aspx
	HANDLE WINAPI CreateFile(
	_In_     LPCTSTR               lpFileName,
	_In_     DWORD                 dwDesiredAccess,
	_In_     DWORD                 dwShareMode,
	_In_opt_ LPSECURITY_ATTRIBUTES lpSecurityAttributes,
	_In_     DWORD                 dwCreationDisposition,
	_In_     DWORD                 dwFlagsAndAttributes,
	_In_opt_ HANDLE                hTemplateFile
	);

	*/
	hComm = CreateFileA(serialPort,
		GENERIC_READ | GENERIC_WRITE,
		0,								//Must be zero
		0,
		OPEN_EXISTING,					//Must specify the OPEN_EXISTING flag
		FILE_ATTRIBUTE_NORMAL,
		NULL);							//Must be NULL


										/*Setup Com Port*/
	if (hComm == INVALID_HANDLE_VALUE) {
		cout << "Error opening port." << endl << "Failed with error: " << GetLastError() << endl;
		return 1;
	}

	if (!SetCommTimeouts(hComm, &timeouts)) {
		cout << "Failed with error: " << GetLastError() << endl;
		return 2;
	}

	hStatus = GetCommState(hComm, &dcb);

	if (!hStatus) {
		cout << "GetCommState failed with error: " << GetLastError() << endl;
		return 3;
	}

	//Setup Serial Connection
	dcb.BaudRate = CBR_57600;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	hStatus = SetCommState(hComm, &dcb);

	if (!hStatus) {
		cout << "SetCommState failed with error: " << GetLastError() << endl;
		return 4;
	}
	else {
		//cout << "Serial Port " << serialPort << " successfully configured" << endl;
	}


	//Write to serial port
	if (sizeof(sendBuf) > maxBufSize) {
		cout << "Message is too long." << endl;
		return 5;
	}

	if (WriteFile(hComm, sendBuf, byteSize, &dwBytesWrite, NULL)) {
		//cout << "Message: \"" << sendBuf << "\" successfully transmitted" << endl;
	}
	//Uncomment to Read Data from Serial Port
	/*
	int16_t szBuf[] = { 0 };
	DWORD dwBytesRead = 0;
	ReadFile(hComm, szBuf, sizeof(szBuf) - 1, &dwBytesRead, NULL);
	*/


	CloseHandle(hComm);
	return 0;


}

//Calculate Homography Matrix for image stitching
int getHomographyMatrix(Mat image1, Mat image2, Mat *H, int minHessian, double minDist = 100.0, bool drawGoodMatches = false)
{
	// TODO : Write Description for getHomographyMatrix

	//Init
	Mat descriptor1, descriptor2, gray1, gray2;
	vector<KeyPoint> keypoint1, keypoint2;
	vector<DMatch> matches, goodMatches;
	vector<Point2f> points1, points2;
	double maxDist = 0;

	//rotate images, so they can be stitched together properly
	flip(image1.t(), image1, 1);
	flip(image2.t(), image2, 0);

	//Convert to grayscale for feature detection
	cvtColor(image1, gray1, CV_BGR2GRAY);
	cvtColor(image2, gray2, CV_BGR2GRAY);

	//initialize SURF Detector
	Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create(minHessian);

	// --> Step 1: Detect the keypoints using SURF Detector
	detector->detect(gray1, keypoint1);
	detector->detect(gray2, keypoint2);

	// --> Step 2: Calculate descriptors (feature vectors)
	detector->compute(gray1, keypoint1, descriptor1);
	detector->compute(gray2, keypoint2, descriptor2);

	// --> Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	matcher.match(descriptor1, descriptor2, matches);


	//--> Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptor1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < minDist)
			minDist = dist;
		if (dist > maxDist)
			maxDist = dist;
	}

	//--> Use only "good" matches, whose distance is less than minDist
	for (int i = 0; i < descriptor1.rows; i++)
	{
		if (matches[i].distance <= 3 * minDist)
		{
			goodMatches.push_back(matches[i]);
		}
	}

	// --> Only show good Matches if drawGodMatches == true
	if (drawGoodMatches)
	{
		Mat imgMatches;
		drawMatches(image1, keypoint1, image2, keypoint2, goodMatches, imgMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		imshow("Good matches", imgMatches);

	}

	//--> Get the keypoints from goodMatches
	for (int i = 0; i < goodMatches.size(); i++)
	{
		points1.push_back(keypoint1[goodMatches[i].queryIdx].pt);
		points2.push_back(keypoint2[goodMatches[i].trainIdx].pt);
	}

	*H = findHomography(points2, points1, CV_RANSAC);

	return ERR_OK;

}

//Image Stitcher
int stitcher(Mat image1, Mat image2, Mat *stitchedImage, Mat H)
{
	// TODO : Write Description for stitcher

	//flip images for proper stitching
	flip(image1.t(), image1, 1);
	flip(image2.t(), image2, 0);

	Mat warped;
	Mat final(Size(image1.cols + image2.cols, image1.rows), CV_8UC3);

	warpPerspective(image2, warped, H, Size(image1.cols + image2.cols, image2.rows));

	Mat roi1(final, Rect(0, 0, image1.cols, image1.rows));
	Mat roi2(final, Rect(0, 0, warped.cols, warped.rows));

	warped.copyTo(roi2);
	image1.copyTo(roi1);

	*stitchedImage = final;
	return ERR_OK;
}

int main(int argc, char *argv[]) {

	//Uncomment to save aruco markers
	/*
	for (int i = 0; i <= 25; i++) {

	string fileName, fileFormat, file;

	int sizeInPixel = 1181;
	int borderBits = 1;

	fileName = to_string(i);
	fileFormat = ".png";
	file = fileName + fileFormat;
	drawArucoMarker(i, sizeInPixel, borderBits, file);

	}
	*/

	//Select Camera Parameter configuration file according to Scaling Factor
	string config_file;// = "calibration_door.xml";

	switch (RES_SCALING_FACTOR)
	{
	case 1: config_file = "camera_distortion_parameters.xml";
		break;
	case 2: config_file = "camera_distortion_parameters_720.xml";
		break;
	case 3: config_file = "camera_distortion_parameters_1080.xml";
		break;
	default:
		break;
	}


	// Open Camera Parameter File
	Mat camMatrix, invCamMatrix, distCoeffs;
	bool readOK = readCameraParameters(config_file, camMatrix, distCoeffs);
	if (!readOK) {
		cerr << "Invalid camera file" << endl;
		return -1;
	}
	invCamMatrix = camMatrix.inv();

	// Set Aruco Dictionary
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

	//Open Webcam
	VideoCapture inputVideo1, inputVideo2;

	//Select Webcams
	inputVideo1.open(2);
	inputVideo2.open(1);

	//Set Resolution
	inputVideo1.set(CAP_PROP_FRAME_HEIGHT, NATIVE_HEIGHT*RES_SCALING_FACTOR);
	inputVideo1.set(CAP_PROP_FRAME_WIDTH, NATIVE_WIDTH*RES_SCALING_FACTOR);

	inputVideo2.set(CAP_PROP_FRAME_HEIGHT, NATIVE_HEIGHT*RES_SCALING_FACTOR);
	inputVideo2.set(CAP_PROP_FRAME_WIDTH, NATIVE_WIDTH*RES_SCALING_FACTOR);

	//initialize Mat objects
	Mat image1, image2, stitchedImage, imageDetected, rotMatrix, invRotMatrix, H;
	inputVideo1.retrieve(image1);
	inputVideo2.retrieve(image2);

	//
	if (RECALCULATE_HOMOGRAPHY)
	{
		getHomographyMatrix(image1, image2, &H, MIN_HESSIAN, 100.0, false);
		// Print new Homography Matrix for saving
		//cout << H;
	}
	else
	{
		// Use "old" Homography Matrix
		H = (Mat_<double>(3, 3) << HOMOGRAPHY_M);
	}

	//Grab frames continuously
	while (inputVideo1.grab() && inputVideo2.grab()) {

		inputVideo1.retrieve(image1);
		inputVideo2.retrieve(image2);

		//stitcher(image1, image2, &stitchedImage, H);

		image1.copyTo(imageDetected);

		//undistort(image1, imageDetected, camMatrix, distCoeffs);

		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners;
		vector<Vec3d> rvecs, tvecs;

		map<int, Mat> marker, cornerBottomLeft;
		map<int, Vec3d> markerTvecs;

		//Detect Markers
		aruco::detectMarkers(imageDetected, dictionary, markerCorners, markerIds);

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

			for (int i = 0; i < markerIds.size(); i++) {

				if (markerIds[i] == (int)ORIGIN_MARKER_ID)
				{
					originIndex = i;
					isOriginVisible = 1;
					coordOrigin = tvecs[originIndex];

					//get Rotation matrix from Rotation vector rvec
					Rodrigues(rvecs[originIndex], rotMatrix);

					//Invert Rotation Matrix once to reduce computational load
					invRotMatrix = rotMatrix.inv();

					//uvOrigin.x = markerCorners[i][0].x;
					//uvOrigin.y = markerCorners[i][0].y;

					//Calculate Center of Origin using moments
					Moments mu = moments(markerCorners[i]);
					uvOrigin = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
					circle(imageDetected, uvOrigin, 5, Scalar(0, 0, 255));

					getWorldCoordinates(uvOrigin, &xyzOrigin, invCamMatrix, invRotMatrix, coordOrigin, 3050.0);
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

						circle(imageDetected, Point2f(640, 360), 5, Scalar(0, 0, 255));

						Mat xyzPoint = Mat::ones(3, 1, DataType<double>::type), relXYZPoint, xyzBottomLeft = Mat::ones(3, 1, DataType<double>::type);
						Point2f uvPoint, uvBottomLeft;

						//Get Real World Coordinates and copy them to marker map
						//Calculate Center using moments
						Moments mu = moments(markerCorners[i]);
						uvPoint = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
						circle(imageDetected, uvPoint, 5, Scalar(0, 0, 255));
						
						//uvPoint.x = markerCorners[i][0].x;
						//uvPoint.y = markerCorners[i][0].y;

						//Get Real World Coordinates of Bottom Left Corner for calculation of Phi
						uvBottomLeft.x = markerCorners[i][3].x;
						uvBottomLeft.y = markerCorners[i][3].y;

						//get Rotation matrix from Rotation vector rvec
						Rodrigues(rvecs[i], rotMatrix);
						//Invert Rotation Matrix once to reduce computational load
						invRotMatrix = rotMatrix.inv();

						Mat centerPoint = Mat::ones(2, 1, DataType<double>::type);
						centerPoint.at<double>(0, 0) = 640;
						centerPoint.at<double>(1, 0) = 360;
						Mat uv = Mat::ones(2, 1, DataType<double>::type);
						uv.at<double>(0, 0) = uvPoint.x;
						uv.at<double>(1, 0) = uvPoint.y;

						double tmp = norm(centerPoint, uv, NORM_L2);
						double zCoord = sqrt(Z_CONST * Z_CONST + tmp*tmp);

						getWorldCoordinates(uvPoint, &xyzPoint, invCamMatrix, invRotMatrix, tvecs[i], Z_CONST);
						getWorldCoordinates(uvBottomLeft, &xyzBottomLeft, invCamMatrix, invRotMatrix, coordOrigin, zCoord);

						cout << "uv: " << uv << endl << "t: " << "Rot: " << rotMatrix; // "xyz: " << xyzPoint << endl << << "Rot: " << rotMatrix << endl << endl;
						//cout << "ID: " << i << ": " << xyzPoint << endl;
						//cout <<  norm( xyzOrigin, xyzPoint, NORM_L2) << endl;

						//Get Relative Coordinates (with Coordinate Origin at OriginMarker Top Left Corner)
						xyzPoint.at<double>(0, 0) = xyzOrigin.at<double>(0, 0) - xyzPoint.at<double>(0, 0);	// rel. x-Coordinate
						xyzPoint.at<double>(1, 0) = xyzOrigin.at<double>(1, 0) - xyzPoint.at<double>(1, 0);	// rel. y-Coordinate

						xyzBottomLeft.at<double>(0, 0) = xyzBottomLeft.at<double>(0, 0) - xyzPoint.at<double>(0, 0);	// rel. x-Coordinate
						xyzBottomLeft.at<double>(1, 0) = xyzBottomLeft.at<double>(1, 0) - xyzPoint.at<double>(1, 0);	// rel. y-Coordinate


						marker[markerIds[i]] = xyzPoint;
						cornerBottomLeft[markerIds[i]] = xyzBottomLeft;
						
					}


				}

				uint16_t message[MAX_MSG_LENGTH];

				for (int i = 0; i < MAX_NUMBER_OF_MARKERS; i++) {

					if (!marker[i].empty()) {
						//Marker with ID = i was detected

						uint16_t phi = (uint16_t)(marker[i].at<double>(0, 0) + cornerBottomLeft[i].at<double>(0, 0));

						message[3 * i] = (uint16_t)(marker[i].at<double>(0, 0)); // x - value
						message[3 * i + 1] = (uint16_t)(marker[i].at<double>(1, 0)); // y - value
						message[3 * i + 2] = phi; // phi - value

					}
					else {
						//Marker with ID = i was NOT detected -> Transmit VAR_INVALID

						message[3 * i] = VAR_INVALID; // x - value
						message[3 * i + 1] = VAR_INVALID; // y - value
						message[3 * i + 2] = VAR_INVALID; // phi; // phi - value
					}

				}


				/*
				for (size_t i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
				{
					if (!marker[i].empty())
						cout << "ID = " << i << " || x = " << (int16_t)message[3 * i] << " | y = " << (int16_t)message[3 * i + 1] << " | phi = " << (int16_t)message[3 * i + 2] << endl;
				}
				*/
				//cout << endl << endl;



				if (SERIAL_TRANSMIT)
				{
					//Send Message to COM Port
					sendSerial("COM2", 255, message, sizeof(message) / sizeof(uint8_t));
				}
				else
				{
					/* Nothing to do */
				}


				//Distance
				/*
				double dist = norm(marker[1], marker[2], NORM_L2);
				*/
			}

		}

		//Draw image
		namedWindow("Detected Markers", WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
		imshow("Detected Markers", imageDetected);

		if (waitKey(10) >= 0) break;

	}

	inputVideo1.release();
	inputVideo2.release();

}


