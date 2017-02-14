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
Std_Rtn_Type drawArucoMarker(int id, int size, int borderBits, string ofileName) {
	/* Draw Aruco Marker
	*/
	Std_Rtn_Type returnCode = ERR_OK;

	Mat markerImage;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	aruco::drawMarker(dictionary, id, size, markerImage, borderBits);

	//namedWindow("Aruco Marker", WINDOW_AUTOSIZE);
	//imshow("Aruco Marker", markerImage);
	imwrite(ofileName, markerImage);
	//waitKey(0);
	cout << ofileName << " Marker succesfully created" << endl;

	return returnCode;
}


//Function to read Camera Parameters from file
Std_Rtn_Type readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	
	Std_Rtn_Type returnCode = ERR_OK;
	
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		returnCode = ERR_INV_PARAM_FILE;
	
	fs["camera_matrix"] >> camMatrix;
	
	fs["distortion_coefficients"] >> distCoeffs;
	
	return returnCode;
}
