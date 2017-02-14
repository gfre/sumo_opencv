#pragma once
/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;

//Get World Coordinates (X,Y,Z) from Image Coordinates (u,v,1)
Std_Rtn_Type getWorldCoordinates(Point2f uv, Mat *xyz, Mat invCamMatrix, Mat invRotMatrix, Vec3d tvec, double zConst);

//Get World Coordinates (X,Y,Z) of Origin Marker
Std_Rtn_Type getOriginXYZ(std::vector<int> *markerIds, int *originIndex_, std::vector<cv::Vec3d> *rvecs, std::vector<cv::Vec3d> *tvecs, cv::Mat *invCamMatrix, std::vector<std::vector<cv::Point2f>> *markerCorners, cv::Mat *xyzOrigin_);

//Get World Coordinates (X,Y,Z) of all Markers
Std_Rtn_Type getMarkerXYZ(std::vector<int> *markerIds, cv::Mat *imageDetected, int originIndex, std::vector<cv::Vec3d> *rvecs, std::vector<cv::Vec3d> *tvecs, cv::Mat *invCamMatrix, cv::Mat *camMatrix, cv::Mat *distCoeffs, std::vector<std::vector<cv::Point2f>> *markerCorners, cv::Mat *xyzOrigin, std::map<int, cv::Mat> *marker_, std::map<int, cv::Mat> *cornerBottomLeft_);
