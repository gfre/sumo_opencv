/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"
#include "opencv2\aruco.hpp"
/* SYSTEM INCLUDES */
#include <string>
#include <iostream>

/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;
using namespace std;

//Function to draw Aruco marker with id, size in pixel, borderBits (must be greater or equal 1), and filename
void drawArucoMarker(int id, int size, int borderBits, string ofileName) {
	/* Draw Aruco Marker
	*/
	Mat markerImage;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	aruco::drawMarker(dictionary, id, size, markerImage, borderBits);

	//namedWindow("Aruco Marker", WINDOW_AUTOSIZE);
	//imshow("Aruco Marker", markerImage);
	imwrite(ofileName, markerImage);
	//waitKey(0);
	cout << ofileName << " Marker succesfully created" << endl;
}


//Function to read Camera Parameters from file
bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}
