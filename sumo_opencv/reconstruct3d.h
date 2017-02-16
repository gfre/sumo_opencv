#pragma once
/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
#include "opencv2\aruco.hpp"
/* PROJECT INCLUDES */
#include "config.h"
/* SYSTEM INCLUDES */
#include <iostream>

//Get World Coordinates (X,Y,Z) from Image Coordinates (u,v,1)
int getWorldCoordinates(cv::Point2f uv, cv::Mat *xyz, cv::Mat invCamMatrix, cv::Mat invRotMatrix, cv::Vec3d tvec, double zConst);

//Get World Coordinates (X,Y,Z) of Origin Marker
int getOriginXYZ(std::vector<int> *markerIds, int *originIndex_, std::vector<cv::Vec3d> *rvecs, std::vector<cv::Vec3d> *tvecs, cv::Mat *invCamMatrix, std::vector<std::vector<cv::Point2f>> *markerCorners, cv::Mat *xyzOrigin_);

//Get World Coordinates (X,Y,Z) of all Markers
int getMarkerXYZ(std::vector<int> *markerIds, cv::Mat *imageDetected, int originIndex, std::vector<cv::Vec3d> *rvecs, std::vector<cv::Vec3d> *tvecs, cv::Mat *invCamMatrix, cv::Mat *camMatrix, cv::Mat *distCoeffs, std::vector<std::vector<cv::Point2f>> *markerCorners, cv::Mat *xyzOrigin, std::map<int, cv::Mat> *marker_, std::map<int, cv::Mat> *cornerBottomLeft_);
