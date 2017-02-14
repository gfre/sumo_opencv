#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

#define CREATE_CHARUCO_BOARD	(1)
#define CALIBRATE_CAMERA		(0)
#define SHOW_CHESSBOARD_CORNERS (1)
#define CALIB_FIX_ASPECT_RATIO	(1)



#define NUM_SQUARES_X			(4)
#define NUM_SQUARES_Y			(6)
#define SQUARE_LENGTH			(397*0+0.14)						// in pixel
#define MARKER_LENGTH			(283*0+0.1)						// in pixel
#define SQUARE_LENGTH_M			(0.04985)					// in meter
#define MARKER_LENGTH_M			(0.0349)					// in meter
#define DICTIONARY_ID			(aruco::DICT_4X4_50)						
#define BORDER_BITS				(1)
#define RES_HEIGHT				(1080)
#define RES_WIDTH				(1920)
#define ASPECT_RATIO			((float)(16/9))

#define REFIND_STRATEGY			(0)

#define FILENAME_CHARUCO		"charuco.png"
#define FILENAME_CALIB			"calibration_door.xml"

#define CAMERA_ID				(1)							//SELECT WHICH CAMERA TO CALIBRATE



int createCharucoBoard(string filename, int squaresX, int squaresY, float squareLength, float markerLength, int dictionaryId, int borderBits)
{

	float margins = squareLength - markerLength;

	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICTIONARY_ID));

	Size imageSize;
	imageSize.width =(( squaresX * squareLength +  2* margins) * 1000 / 25.4 * 72)*0+2834.645669;
	imageSize.height = ((squaresY * squareLength + 2*margins)*1000/25.4*72) * 0 + 2834.645669;

	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength, (float)markerLength, dictionary);

	// show created board
	Mat boardImage;
	board->draw(imageSize, boardImage, margins, borderBits);

	imshow("board", boardImage);
	imwrite(filename, boardImage);

	waitKey(0);

	return 0;
}

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params)
{
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["doCornerRefinement"] >> params->doCornerRefinement;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	return true;
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



int main(int argc, char *argv[]) {

#if CREATE_CHARUCO_BOARD
		createCharucoBoard(FILENAME_CHARUCO, NUM_SQUARES_X, NUM_SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH, DICTIONARY_ID, BORDER_BITS);
#endif
	
#if CALIBRATE_CAMERA
	
		int calibrationFlags = 0;
		float aspectRatio = 1;

		Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

		VideoCapture inputVideo;
		inputVideo.open(CAMERA_ID);

		//Set Resolution
		inputVideo.set(CAP_PROP_FRAME_HEIGHT, RES_HEIGHT);
		inputVideo.set(CAP_PROP_FRAME_WIDTH, RES_WIDTH);

		Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICTIONARY_ID));

		// create charuco board object
		Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(NUM_SQUARES_X, NUM_SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH, dictionary);
		Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

		// collect data from each frame
		vector< vector< vector< Point2f > > > allCorners;
		vector< vector< int > > allIds;
		vector< Mat > allImgs;
		Size imgSize;

		while (inputVideo.grab()) 
		{
			Mat image, imageCopy;
			inputVideo.retrieve(image);

			vector< int > ids;
			vector< vector< Point2f > > corners, rejected;

			// detect markers
			aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

			// refind strategy to detect more markers
			if (REFIND_STRATEGY) 
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
			/*
			if (key == 'c' && ids.size() > 0) 
			{
				cout << "Frame captured" << endl;
				allCorners.push_back(corners);
				allIds.push_back(ids);
				allImgs.push_back(image);
				imgSize = image.size();
			}
			*/
			if (ids.size() > 10)
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
			return 0;
		}

		Mat cameraMatrix, distCoeffs;
		vector< Mat > rvecs, tvecs;
		double repError;
/*
		if (CALIB_FIX_ASPECT_RATIO) {
			cameraMatrix = Mat::eye(3, 3, CV_64F);
			cameraMatrix.at< double >(0, 0) = ASPECT_RATIO;
		}
*/		
		cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at< double >(0, 0) = 900.;
		cameraMatrix.at< double >(0, 2) = 960.;
		cameraMatrix.at< double >(1, 1) = 900.;
		cameraMatrix.at< double >(1, 2) = 540.;
		
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
		arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated, markerCounterPerFrame, board, imgSize, cameraMatrix, distCoeffs, noArray(), noArray(), CV_CALIB_USE_INTRINSIC_GUESS);

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
			return 0;
		}

		// calibrate camera using charuco
		repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

		bool saveOk = saveCameraParams(FILENAME_CALIB, imgSize, aspectRatio, calibrationFlags, cameraMatrix, distCoeffs, repError);
		if (!saveOk) {
			cerr << "Cannot save output file" << endl;
			return 0;
		}

		cout << "Rep Error: " << repError << endl;
		cout << "Rep Error Aruco: " << arucoRepErr << endl;
		cout << "Calibration saved to " << FILENAME_CALIB << endl;

		// show interpolated charuco corners for debugging
		if (SHOW_CHESSBOARD_CORNERS) {
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

		return 0;

	
#endif

	return 0;
}