/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
#include "opencv2\aruco.hpp"
/* PROJECT INCLUDES */
#include "config.h"
#include "fileOutput.h"
/* SYSTEM INCLUDES */
#include <iostream>
#include <cmath>

//Get All Euler Angles in Radians from RotMatrix
int getAllEulerAnglesFromRotMatrix(std::vector<cv::Vec3d> &rvecs, std::vector<int> &markerIds, std::map<int, double> &zAngle_, int originIndex)
{
	for (size_t i = 0; i < (markerIds.size()); i++)
	{
		if (i != originIndex)
		{
			double xAngle;
			double sinX, cosX;
			cv::Mat rotMatrix;

			cv::Rodrigues(rvecs[i], rotMatrix);

			xAngle = atan2(rotMatrix.at<double>(2, 1), rotMatrix.at<double>(2, 2));

			sinX = sin(xAngle);
			cosX = cos(xAngle);

			zAngle_[markerIds[i]] = atan2(sinX*rotMatrix.at<double>(0, 2) - cosX*rotMatrix.at<double>(0, 1), cosX*rotMatrix.at<double>(1, 1) - sinX*rotMatrix.at<double>(1, 2));

		}

	}

	return ERR_OK;
}

//Get Euler Angle in Radians from RotMatrix
int getEulerAngleFromRotMatrix(cv::Mat &rotMatrix, int &markerId, std::map<int, double> &zAngle_)
{
	double xAngle;
	double sinX, cosX;

	xAngle = atan2(rotMatrix.at<double>(2, 1), rotMatrix.at<double>(2, 2));

	sinX = sin(xAngle);
	cosX = cos(xAngle);

	zAngle_[markerId] = atan2(sinX*rotMatrix.at<double>(0, 2) - cosX*rotMatrix.at<double>(0, 1), cosX*rotMatrix.at<double>(1, 1) - sinX*rotMatrix.at<double>(1, 2));

	return ERR_OK;
}

//Get World Coordinates (X,Y,Z) from Image Coordinates (u,v,1)
int getWorldCoordinates(cv::Point2f uv, cv::Mat &xyz, cv::Mat invCamMatrix, cv::Mat invRotMatrix, cv::Vec3d tvec, double zConst) 
{

	int returnCode = ERR_OK;

	double s;
	cv::Mat uvPoint, t, tmp, tmp2;

	uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	uvPoint.at<double>(0, 0) = uv.x;
	uvPoint.at<double>(1, 0) = uv.y;

	//Copy translation vector tvec to cv::Mat object t
	t = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	t.at<double>(0, 0) = tvec[0];
	t.at<double>(1, 0) = tvec[1];
	t.at<double>(2, 0) = tvec[2];


#if ROTATE_FIRST 

	//calculate temp values to solve eq for scaling factor s
	tmp = invRotMatrix * (invCamMatrix * uvPoint);
	tmp2 = (invRotMatrix * t);

	s = (zConst + tmp2.at<double>(2, 0)) / tmp.at<double>(2, 0);

	xyz = invRotMatrix *( (s*invCamMatrix * uvPoint) - t);
	double zk = s* tmp.at<double>(2, 0) - tmp2.at<double>(2, 0);
	double y  = s* tmp.at<double>(1, 0) - tmp2.at<double>(1, 0);
	double x =  s* tmp.at<double>(0, 0) - tmp2.at<double>(0, 0);
#else

	tmp = invRotMatrix * (invCamMatrix * uvPoint);
	tmp2 =  t;
	s = (zConst - t.at<double>(2, 0)) / tmp.at<double>(2, 0);
	xyz = (invRotMatrix * s * (invCamMatrix * uvPoint)) + t;
#endif



#if 0
	std::cout << "uv: " << uv.x << "  " << uv.y << std::endl;
	std::cout << "t: " << t << std::endl;
	std::cout << "invRot: " << invRotMatrix << std::endl; 
	std::cout << "iRM3: " << tmp.at<double>(2, 0) << std::endl;
	std::cout << "iRt3: " << tmp2.at<double>(2, 0) << std::endl;
	std::cout << "x: " << x << std::endl;
	std::cout << "y: " << y << std::endl;
	std::cout << "zk: " << zk << std::endl;
	std::cout << (invRotMatrix*(s*invCamMatrix * uvPoint) - t) << std::endl << std::endl << std::endl;
#endif

	return returnCode;

}

//Get World Coordinates (X,Y,Z) of Origin Marker
int getOriginXYZ(std::vector<int> &markerIds, int &originIndex_, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, cv::Mat &invCamMatrix, std::vector<std::vector<cv::Point2f>> &markerCorners, cv::Mat &xyzOrigin_)
{
	int errorCode = ERR_OK;
	cv::Point2d uvOrigin;
	int isOriginVisible = 0;

	for (int i = 0; i < markerIds.size(); i++)
	{
		if (markerIds[i] == ORIGIN_MARKER_ID)
		{
			originIndex_ = i;
			isOriginVisible = 1;
			cv::Vec3d coordOrigin = tvecs[originIndex_];
			cv::Mat rotMatrix;

			//get Rotation cv::Matrix from Rotation vector rvec
			Rodrigues(rvecs[originIndex_], rotMatrix);
			//Invert Rotation cv::Matrix
			cv::Mat invRotMatrix = rotMatrix.inv();


#if SHIFT_POINT_TO_CENTER
			//Calculate Center of Origin using moments
			cv::Moments mu = cv::moments(markerCorners[i]);
			uvOrigin = cv::Point2d(mu.m10 / mu.m00, mu.m01 / mu.m00);
			
#else
			//Use top left corner
			uvOrigin.x = markerCorners[i][0].x;
			uvOrigin.y = markerCorners[i][0].y;
#endif

#if PRINT_UV_COORDS 
			std::cout << "Origin Marker (ID: " << markerIds[i] << ")" << std::endl;
			std::cout << "------------------------------------------" << std::endl;
			std::cout << "U: " << uvOrigin.x << " V: " << uvOrigin.y << std::endl;
			std::cout << "Rotcv::Matrix: " << rotMatrix << std::endl;
			std::cout << "Transl: " << tvecs[i] << std::endl;
			std::cout << "============================================" << std::endl << std::endl << std::endl;
#endif
			getWorldCoordinates(uvOrigin, xyzOrigin_, invCamMatrix, invRotMatrix, coordOrigin, Z_CONST);

#if PRINT_ORIGIN_COORDS

			std::cout << "Origin Marker ID: " << markerIds[i] << " - X:" << xyzOrigin_.at<double>(0, 0) << " Y: " << xyzOrigin_.at<double>(1, 0) << " Z: " << xyzOrigin_.at<double>(2, 0) << std::endl;
#endif
		}
	}

	if (!isOriginVisible)
		errorCode = ERR_NO_ORIGIN;

	return errorCode;

}

//Get World Coordinates (X,Y,Z) of all Markers
int getAllMarkerXYZ(std::vector<int> &markerIds, cv::Mat &imageDetected, int originIndex, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, cv::Mat &invCamMatrix, cv::Mat &camMatrix, cv::Mat &distCoeffs, std::vector<std::vector<cv::Point2f>> &markerCorners, cv::Mat &xyzOrigin, std::map<int, cv::Mat> &marker_)
{
	int errorCode = ERR_OK;

	cv::Mat rotMatrix;
	cv::Mat xyzPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	cv::Point2f uvPoint;

	//Save all detected markers in map "marker"
	for (size_t i = 0; i < (markerIds.size()); i++)
	{
		if (i == originIndex) {
			cv::aruco::drawAxis(imageDetected, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 0.5f);
		}
		else
		{
			cv::aruco::drawAxis(imageDetected, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH * 0.5f);
			//get Rotation cv::Matrix from Rotation vector rvec
			cv::Rodrigues(rvecs[i], rotMatrix);
			//Invert Rotation cv::Matrix once to reduce computational load
			cv::Mat invRotMatrix = rotMatrix.inv();
			
#if SHIFT_POINT_TO_CENTER
			//Calculate Center using moments
			cv::Moments mu = moments(markerCorners[i]);
			uvPoint = cv::Point2d(mu.m10 / mu.m00, mu.m01 / mu.m00);
			cv::circle(imageDetected, uvPoint, 5, cv::Scalar(0, 255, 0));
#else
			//Use top left corner
			uvPoint.x = markerCorners[i][0].x;
			uvPoint.y = markerCorners[i][0].y;
#endif

#if PRINT_UV_COORDS 
			std::cout << "ID: " << markerIds[i] << std::endl;
			std::cout << "------------------------------------------" << std::endl;
			std::cout << "U: " << uvPoint.x << " V: " << uvPoint.y << std::endl;
			std::cout << "RotMatrix: " << rotMatrix << std::endl;
			std::cout << "Transl: " << tvecs[i] << std::endl;

			std::cout << "============================================" << std::endl << std::endl << std::endl;
#endif


			getWorldCoordinates(uvPoint, xyzPoint, invCamMatrix, invRotMatrix, tvecs[i], Z_CONST);
			
#if USE_REL_COORDS
			double dist = cv::norm(xyzOrigin, xyzPoint, cv::NORM_L2);
			//Get Relative Coordinates (with Coordinate Origin at OriginMarker Top Left Corner)
			xyzPoint.at<double>(0, 0) -= xyzOrigin.at<double>(0, 0);	// rel. x-Coordinate
			xyzPoint.at<double>(1, 0) -= xyzOrigin.at<double>(1, 0);	// rel. y-Coordinate
			
#endif

			marker_[markerIds[i]] = xyzPoint;
			
		}
	}

	return errorCode;
}

//Get World Coordinates (X,Y,Z) of one marker
int getMarkerXYZ(int markerId, cv::Mat &invRotMatrix, std::vector<cv::Point2f> &markerCorners, cv::Vec3d &transl, cv::Mat &invCamMatrix, std::map<int, cv::Mat> &marker_, cv::Mat &xyzOrigin)
{
	int errorCode = ERR_OK;

	cv::Mat xyzPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	cv::Point2f uvPoint;


#if SHIFT_POINT_TO_CENTER
	//Calculate Center using moments
	cv::Moments mu = cv::moments(markerCorners);
	uvPoint = cv::Point2d(mu.m10 / mu.m00, mu.m01 / mu.m00);
#else
	//Use top left corner
	uvPoint.x = markerCorners[0].x;
	uvPoint.y = markerCorners[0].y;
#endif

#if PRINT_UV_COORDS 
	std::cout << "ID: " << markerId << std::endl;
	std::cout << "------------------------------------------" << std::endl;
	std::cout << "U: " << uvPoint.x << " V: " << uvPoint.y << std::endl;
	std::cout << "invRotMatrix: " << invRotMatrix << std::endl;
	std::cout << "Transl: " << transl << std::endl;

	std::cout << "============================================" << std::endl << std::endl << std::endl;
#endif


	getWorldCoordinates(uvPoint, xyzPoint, invCamMatrix, invRotMatrix, transl, Z_CONST);

#if USE_REL_COORDS
	double dist = cv::norm(xyzOrigin, xyzPoint, cv::NORM_L2);

	xyzPoint.at<double>(0, 0) -= xyzOrigin.at<double>(0, 0);	// rel. x-Coordinate
	xyzPoint.at<double>(1, 0) -= xyzOrigin.at<double>(1, 0);	// rel. y-Coordinate

#endif


	marker_[markerId] = xyzPoint;

return errorCode;
}

