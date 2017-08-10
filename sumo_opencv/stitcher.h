#pragma once
/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;

//Calculate Homography Matrix for image stitching
int getHomographyMatrix(Mat image1, Mat image2, Mat *H, int minHessian, double minDist = 100.0, bool drawGoodMatches = false);

//Image Stitcher
int stitcher(Mat image1, Mat image2, Mat *stitchedImage, Mat H);
