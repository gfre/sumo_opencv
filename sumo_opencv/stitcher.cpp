#include "opencv2\highgui.hpp"
#include "opencv2\core.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\calib3d.hpp"
#include "opencv2\xfeatures2d.hpp"

#include <iostream>

#include "config.h"

using namespace cv;
using namespace std;

//Calculate Homography Matrix for image stitching
int getHomographyMatrix(Mat image1, Mat image2, Mat *H, int minHessian, double minDist = 100.0, bool drawGoodMatches = false)
{
	// TODO : Write Description for getHomographyMatrix

	//Init
	Mat descriptor1, descriptor2, gray1, gray2;
	vector<KeyPoint> keypoint1, keypoint2;
	vector<DMatch> matches, goodMatches;
	vector<Point2f> points1, points2;
	double maxDist = 0;

	//rotate images, so they can be stitched together properly
	flip(image1.t(), image1, 1);
	flip(image2.t(), image2, 0);

	//Convert to grayscale for feature detection
	cvtColor(image1, gray1, CV_BGR2GRAY);
	cvtColor(image2, gray2, CV_BGR2GRAY);

	//initialize SURF Detector
	Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create(minHessian);

	// --> Step 1: Detect the keypoints using SURF Detector
	detector->detect(gray1, keypoint1);
	detector->detect(gray2, keypoint2);

	// --> Step 2: Calculate descriptors (feature vectors)
	detector->compute(gray1, keypoint1, descriptor1);
	detector->compute(gray2, keypoint2, descriptor2);

	// --> Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	matcher.match(descriptor1, descriptor2, matches);


	//--> Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptor1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < minDist)
			minDist = dist;
		if (dist > maxDist)
			maxDist = dist;
	}

	//--> Use only "good" matches, whose distance is less than minDist
	for (int i = 0; i < descriptor1.rows; i++)
	{
		if (matches[i].distance <= 3 * minDist)
		{
			goodMatches.push_back(matches[i]);
		}
	}

	// --> Only show good Matches if drawGodMatches == true
	if (drawGoodMatches)
	{
		Mat imgMatches;
		drawMatches(image1, keypoint1, image2, keypoint2, goodMatches, imgMatches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		imshow("Good matches", imgMatches);

	}

	//--> Get the keypoints from goodMatches
	for (int i = 0; i < goodMatches.size(); i++)
	{
		points1.push_back(keypoint1[goodMatches[i].queryIdx].pt);
		points2.push_back(keypoint2[goodMatches[i].trainIdx].pt);
	}

	*H = findHomography(points2, points1, CV_RANSAC);

	return ERR_OK;

}

//Image Stitcher
int stitcher(Mat image1, Mat image2, Mat *stitchedImage, Mat H)
{
	// TODO : Write Description for stitcher

	//flip images for proper stitching
	flip(image1.t(), image1, 1);
	flip(image2.t(), image2, 0);

	Mat warped;
	Mat final(Size(image1.cols + image2.cols, image1.rows), CV_8UC3);

	warpPerspective(image2, warped, H, Size(image1.cols + image2.cols, image2.rows));

	Mat roi1(final, Rect(0, 0, image1.cols, image1.rows));
	Mat roi2(final, Rect(0, 0, warped.cols, warped.rows));

	warped.copyTo(roi2);
	image1.copyTo(roi1);

	*stitchedImage = final;
	return ERR_OK;
}