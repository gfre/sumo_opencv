/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"
#include "opencv2\aruco.hpp"
#include <opencv2/aruco/charuco.hpp>
/* SYSTEM INCLUDES */
#include <string>
#include <iostream>

/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;
using namespace std;

//Function to draw Aruco marker with id, size in pixel, borderBits (must be greater or equal 1), and filename
int drawArucoMarker(int id, int size, int borderBits, string ofileName) {
	/* Draw Aruco Marker
	*/
	int returnCode = ERR_OK;

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
int readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	
	int returnCode = ERR_OK;
	
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		returnCode = ERR_INV_PARAM_FILE;
	
	fs["camera_matrix"] >> camMatrix;
	
	fs["distortion_coefficients"] >> distCoeffs;
	
	return returnCode;
}

//Function to generate Aruco Codes
int generateAruco()
{
	for (int i = 0; i <= 25; i++) {
		string fileName, fileFormat, file;

		/* Calculate Pixel --> cm
		https://www.blitzrechner.de/pixel-zentimeter-umrechnen/
		@300dpi - 118px/cm
		*/
		int sizeInPixel = 1772;	// = 15cm @ 300dpi
		int borderBits = 2;

		fileName = to_string(i);
		fileFormat = ".png";
		file = "aruco_codes/" + fileName + fileFormat;
		drawArucoMarker(i, sizeInPixel, borderBits, file);
	}

	return ERR_OK;
}

int createCharucoBoard(string filename, int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId, int borderBits)
{

	float margins = squareLength - markerLength;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(ARUCO_DICT));

	Size imageSize;
	imageSize.width = ((squaresX * squareLength + 2 * margins) * 1000 / 25.4 * 72) * 0 + 2834.645669;
	imageSize.height = ((squaresY * squareLength + 2 * margins) * 1000 / 25.4 * 72) * 0 + 2834.645669;

	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength, (float)markerLength, dictionary);

	// show created board
	Mat boardImage;
	board->draw(imageSize, boardImage, margins, borderBits);

	imshow("board", boardImage);
	imwrite(filename, boardImage);

	waitKey(0);

	return 0;
}

static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags, const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened())
		return false;

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;

	if (flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;

	return true;
}

int calibrateCamera()
{
	int errorCode = ERR_OK;
	float aspectRatio = CHARUCO_ASPECT_RATIO;
	int calibrationFlags = CHARUCO_FLAGS;

	//Corner Refinement
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	detectorParams->doCornerRefinement = true;
	detectorParams->cornerRefinementWinSize = 5;
	detectorParams->cornerRefinementMaxIterations = 50;
	detectorParams->cornerRefinementMinAccuracy = 0.01;

	VideoCapture inputVideo;
	inputVideo.open(CHARUCO_CAM_ID);

	//Set Resolution
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	inputVideo.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);


	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(ARUCO_DICT));

	// create charuco board object
	Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(CHARUCO_NUM_SQUARES_X, CHARUCO_NUM_SQUARES_Y, CHARUCO_SQUARE_LENGTH, CHARUCO_MARKER_LENGTH, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

	// collect data from each frame
	vector< vector< vector< Point2f > > > allCorners;
	vector< vector< int > > allIds;
	vector< Mat > allImgs;
	Size imgSize;

	Mat cameraMatrix = Mat::zeros(3, 3, CV_64F);

#if CHARUCO_USE_INTRINSIC_GUESS
		cameraMatrix.at< double >(0, 0) = CHARUCO_FOCAL_LENGTH_EST;
		cameraMatrix.at< double >(0, 2) = FRAME_WIDTH / 2;
		cameraMatrix.at< double >(1, 1) = CHARUCO_FOCAL_LENGTH_EST;
		cameraMatrix.at< double >(1, 2) = FRAME_HEIGHT / 2;
		cameraMatrix.at< double >(2, 2) = 1;
#endif

		while (inputVideo.grab())
		{
			Mat image, imageCopy;
			inputVideo.retrieve(image);

			vector< int > ids;
			vector< vector< Point2f > > corners, rejected;

			// detect markers
			aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

			// refind strategy to detect more markers
			if (CHARUCO_REFIND_STRATEGY)
				aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

			// interpolate charuco corners
			Mat currentCharucoCorners, currentCharucoIds;
			if (ids.size() > 0)
				aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners, currentCharucoIds);

			// draw results
			image.copyTo(imageCopy);
			if (ids.size() > 0)
				aruco::drawDetectedMarkers(imageCopy, corners);

			if (currentCharucoCorners.total() > 0)
				aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

			putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
				Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

			imshow("out", imageCopy);
			char key = (char)waitKey(10);

			if (key == 27) break;

			if (key == 'c' && ids.size() > 0)
			{
				cout << "Frame captured" << endl;
				allCorners.push_back(corners);
				allIds.push_back(ids);
				allImgs.push_back(image);
				imgSize = image.size();
			}
		}


		if (allIds.size() < 1)
		{
			cerr << "Not enough captures for calibration" << endl;
			return CHARUCO_ERR_NOT_ENOUGH_FRAMES;
		}

		Mat distCoeffs;
		vector< Mat > rvecs, tvecs;
		double repError;


		// prepare data for calibration
		vector< vector< Point2f > > allCornersConcatenated;
		vector< int > allIdsConcatenated;
		vector< int > markerCounterPerFrame;
		markerCounterPerFrame.reserve(allCorners.size());
		for (unsigned int i = 0; i < allCorners.size(); i++) {
			markerCounterPerFrame.push_back((int)allCorners[i].size());
			for (unsigned int j = 0; j < allCorners[i].size(); j++) {
				allCornersConcatenated.push_back(allCorners[i][j]);
				allIdsConcatenated.push_back(allIds[i][j]);
			}
		}

		// calibrate camera using aruco markers

		double arucoRepErr;
		arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, board, imgSize, cameraMatrix, distCoeffs, noArray(), noArray(), calibrationFlags);

		// prepare data for charuco calibration
		int nFrames = (int)allCorners.size();
		vector< Mat > allCharucoCorners;
		vector< Mat > allCharucoIds;
		vector< Mat > filteredImages;
		allCharucoCorners.reserve(nFrames);
		allCharucoIds.reserve(nFrames);

		for (int i = 0; i < nFrames; i++)
		{
			// interpolate using camera parameters
			Mat currentCharucoCorners, currentCharucoIds;
			aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard, currentCharucoCorners, currentCharucoIds, cameraMatrix, distCoeffs);

			allCharucoCorners.push_back(currentCharucoCorners);
			allCharucoIds.push_back(currentCharucoIds);
			filteredImages.push_back(allImgs[i]);
		}


		if (allCharucoCorners.size() < 4)
		{
			cerr << "Not enough corners for calibration" << endl;
			return CHARUCO_ERR_NOT_ENOUGH_CORNERS;
		}

		// calibrate camera using charuco
		repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

		bool saveOk = saveCameraParams(CHARUCO_FILENAME_CALIB, imgSize, aspectRatio, calibrationFlags, cameraMatrix, distCoeffs, repError);
		if (!saveOk) {
			cerr << "Cannot save output file" << endl;
			return CHARUCO_ERR_CALIB_FILE;
		}

#if CHARUCO_PRINT_FINAL
		cout << "Rep Error: " << repError << endl;
		cout << "Rep Error Aruco: " << arucoRepErr << endl;
		cout << "Calibration saved to " << CHARUCO_FILENAME_CALIB << endl;
#endif

		// show interpolated charuco corners for debugging
		if (CHARUCO_SHOW_CHESSBOARD_CORNERS) {
			for (unsigned int frame = 0; frame < filteredImages.size(); frame++) {
				Mat imageCopy = filteredImages[frame].clone();
				if (allIds[frame].size() > 0) {

					if (allCharucoCorners[frame].total() > 0) {
						aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame],
							allCharucoIds[frame]);
					}
				}

				imshow("out", imageCopy);
				char key = (char)waitKey(0);
				if (key == 27) break;
			}
		}

	return errorCode;
}
