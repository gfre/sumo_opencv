#pragma once
/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;

//Calculate Homography Matrix for image stitching
Std_Rtn_Type getHomographyMatrix(Mat image1, Mat image2, Mat *H, int minHessian, double minDist = 100.0, bool drawGoodMatches = false);

//Image Stitcher
Std_Rtn_Type stitcher(Mat image1, Mat image2, Mat *stitchedImage, Mat H);