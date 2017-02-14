/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
#include "opencv2\aruco.hpp"
/* PROJECT INCLUDES */
#include "config.h"
/* SYSTEM INCLUDES */
#include <iostream>


//Get World Coordinates (X,Y,Z) from Image Coordinates (u,v,1)
Std_Rtn_Type getWorldCoordinates(cv::Point2f uv, cv::Mat *xyz, cv::Mat invCamMatrix, cv::Mat invRotMatrix, cv::Vec3d tvec, double zConst) 
{

	Std_Rtn_Type returnCode = ERR_OK;

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
	tmp = invRotMatrix * invCamMatrix * uvPoint;
	tmp2 = invRotMatrix * t;

	s = (zConst + tmp2.at<double>(2, 0)) / tmp.at<double>(2, 0);

	*xyz = (invRotMatrix * (s * invCamMatrix * uvPoint) - t);

#if 0
	std::cout << "uv: " << uv.x << "  " << uv.y << std::endl;
	std::cout << "t: " << t << std::endl;
	std::cout << "Rot:Matrix: " << invRotMatrix << std::endl;
	std::cout << (invRotMatrix*(s*invCamMatrix * uvPoint)-t) << std::endl << std::endl << std::endl;
#endif

#else

	tmp = invRotMatrix * invCamMatrix * uvPoint;
	s = (zConst + t.at<double>(2, 0)) / tmp.at<double>(2, 0);
	*xyz = (invRotMatrix * s * invCamMatrix * uvPoint) - t;
#endif


	return returnCode;

}

//Get World Coordinates (X,Y,Z) of Origin Marker
Std_Rtn_Type getOriginXYZ(std::vector<int> *markerIds, int *originIndex_, std::vector<cv::Vec3d> *rvecs, std::vector<cv::Vec3d> *tvecs, cv::Mat *invCamMatrix, std::vector<std::vector<cv::Point2f>> *markerCorners, cv::Mat *xyzOrigin_)
{
	Std_Rtn_Type errorCode = ERR_OK;
	cv::Point2f uvOrigin;
	int isOriginVisible = 0;

	for (int i = 0; i < markerIds->size(); i++)
	{
		if ((*markerIds)[i] == ORIGIN_MARKER_ID)
		{
			*originIndex_ = i;
			isOriginVisible = 1;
			cv::Vec3d coordOrigin = (*tvecs)[*originIndex_];
			cv::Mat rotMatrix;

			//get Rotation cv::Matrix from Rotation vector rvec
			Rodrigues((*rvecs)[*originIndex_], rotMatrix);
			//Invert Rotation cv::Matrix
			cv::Mat invRotMatrix = rotMatrix.inv();


#if SHIFT_POINT_TO_CENTER
			//Calculate Center of Origin using moments
			cv::Moments mu = cv::moments((*markerCorners)[i]);
			uvOrigin = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
#else
			//Use top left corner
			uvOrigin.x = (*markerCorners)[i][0].x;
			uvOrigin.y = (*markerCorners)[i][0].y;
#endif

#if PRINT_UV_COORDS 
			std::cout << "Origin Marker (ID: " << (*markerIds)[i] << ")" << std::endl;
			std::cout << "------------------------------------------" << std::endl;
			std::cout << "U: " << uvOrigin.x << " V: " << uvOrigin.y << std::endl;
			std::cout << "Rotcv::Matrix: " << rotMatrix << std::endl;
			std::cout << "Transl: " << (*tvecs)[i] << std::endl;
			std::cout << "============================================" << std::endl << std::endl << std::endl;
#endif
			getWorldCoordinates(uvOrigin, xyzOrigin_, *invCamMatrix, invRotMatrix, coordOrigin, Z_CONST);

#if PRINT_WOLRD_COORDS 

			std::cout << "Origin Marker ID: " << (*markerIds)[i] << " - X:" << (*xyzOrigin_).at<double>(0, 0) << " Y: " << (*xyzOrigin_).at<double>(1, 0) << " Z: " << (*xyzOrigin_).at<double>(2, 0) << std::endl;
#endif
		}
	}

	if (!isOriginVisible)
		errorCode = ERR_NO_ORIGIN;

	return errorCode;

}

//Get World Coordinates (X,Y,Z) of all Markers
Std_Rtn_Type getMarkerXYZ(std::vector<int> *markerIds, cv::Mat *imageDetected, int originIndex, std::vector<cv::Vec3d> *rvecs, std::vector<cv::Vec3d> *tvecs, cv::Mat *invCamMatrix, cv::Mat *camMatrix, cv::Mat *distCoeffs, std::vector<std::vector<cv::Point2f>> *markerCorners, cv::Mat *xyzOrigin, std::map<int, cv::Mat> *marker_, std::map<int, cv::Mat> *cornerBottomLeft_)
{
	Std_Rtn_Type errorCode = ERR_OK;

	cv::Mat rotMatrix;
	cv::Mat xyzPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	cv::Mat xyzBottomLeft = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	cv::Point2f uvPoint, uvBottomLeft;

	//Save all detected markers in map "marker"
	for (unsigned int i = 0; i < (markerIds->size()); i++)
	{
		if (i == originIndex) {
			cv::aruco::drawAxis(*imageDetected, *camMatrix, *distCoeffs, (*rvecs)[i], (*tvecs)[i], MARKER_LENGTH * 0.5f);
		}
		else
		{
			cv::aruco::drawAxis(*imageDetected, *camMatrix, *distCoeffs, (*rvecs)[i], (*tvecs)[i], MARKER_LENGTH * 0.5f);
			//get Rotation cv::Matrix from Rotation vector rvec
			cv::Rodrigues((*rvecs)[i], rotMatrix);
			//Invert Rotation cv::Matrix once to reduce computational load
			cv::Mat invRotMatrix = rotMatrix.inv();

#if SHIFT_POINT_TO_CENTER
			//Calculate Center using moments
			cv::Moments mu = moments((*markerCorners)[i]);
			uvPoint = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
			cv::circle(*imageDetected, uvPoint, 5, cv::Scalar(0, 255, 0));
#else
			//Use top left corner
			uvPoint.x = (*markerCorners)[i][0].x;
			uvPoint.y = (*markerCorners)[i][0].y;
#endif

			//Get Real World Coordinates of Bottom Left Corner for calculation of Phi
			uvBottomLeft.x = (*markerCorners)[i][3].x;
			uvBottomLeft.y = (*markerCorners)[i][3].y;

			//get Rotation cv::Matrix from Rotation vector rvec
			Rodrigues((*rvecs)[i], rotMatrix);
			//Invert Rotation cv::Matrix once to reduce computational load
			invRotMatrix = rotMatrix.inv();

			cv::Mat uv = cv::Mat::ones(2, 1, cv::DataType<double>::type);
			uv.at<double>(0, 0) = uvPoint.x;
			uv.at<double>(1, 0) = uvPoint.y;

#if PRINT_UV_COORDS 
			std::cout << "ID: " << (*markerIds)[i] << std::endl;
			std::cout << "------------------------------------------" << std::endl;
			std::cout << "U: " << uvPoint.x << " V: " << uvPoint.y << std::endl;
			std::cout << "RotMatrix: " << rotMatrix << std::endl;
			std::cout << "Transl: " << (*tvecs)[i] << std::endl;

			std::cout << "============================================" << std::endl << std::endl << std::endl;
#endif


			getWorldCoordinates(uvPoint, &xyzPoint, *invCamMatrix, invRotMatrix, (*tvecs)[i], Z_CONST);
			getWorldCoordinates(uvBottomLeft, &xyzBottomLeft, *invCamMatrix, invRotMatrix, (*tvecs)[i], Z_CONST);

			//cout <<  norm( xyzOrigin, xyzPoint, NORM_L2) << endl;
#if USE_REL_COORDS
			//Get Relative Coordinates (with Coordinate Origin at OriginMarker Top Left Corner)
			xyzPoint.at<double>(0, 0) -= (*xyzOrigin).at<double>(0, 0);	// rel. x-Coordinate
			xyzPoint.at<double>(1, 0) -= (*xyzOrigin).at<double>(1, 0);	// rel. y-Coordinate

			xyzBottomLeft.at<double>(0, 0) -= xyzPoint.at<double>(0, 0);	// rel. x-Coordinate
			xyzBottomLeft.at<double>(1, 0) -= xyzPoint.at<double>(1, 0);	// rel. y-Coordinate
#endif
			(*marker_)[(*markerIds)[i]] = xyzPoint;
			(*cornerBottomLeft_)[(*markerIds)[i]] = xyzBottomLeft;

#if PRINT_WOLRD_COORDS 

			std::cout << "Marker ID: " << (*markerIds)[i] << " - X:" << (*marker_)[(*markerIds)[i]].at<double>(0, 0) << " Y: " << (*marker_)[(*markerIds)[i]].at<double>(1, 0) << " Z: " << (*marker_)[(*markerIds)[i]].at<double>(2, 0) << std::endl;
#endif

		}
	}

	return errorCode;
}


