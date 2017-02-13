#pragma once
/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"

using namespace cv;

int getWorldCoordinates(Point2f uv, Mat *xyz, Mat invCamMatrix, Mat invRotMatrix, Vec3d tvec, double zConst);