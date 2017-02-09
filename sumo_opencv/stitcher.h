#pragma once
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"

using namespace cv;

//Calculate Homography Matrix for image stitching
int getHomographyMatrix(Mat image1, Mat image2, Mat *H, int minHessian, double minDist = 100.0, bool drawGoodMatches = false);

//Image Stitcher
int stitcher(Mat image1, Mat image2, Mat *stitchedImage, Mat H);