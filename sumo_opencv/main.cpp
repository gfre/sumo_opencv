/*
Author : Nico Engel, nice@tf.uni-kiel.de, CAU-Kiel
Date   : 17.02.2017
*/

/* PROJECT INCLUDES */
#include "config.h"
#include "arucoFunc.h"
#include "detect.h"

#define MODE_GENERATE_ARUCO_CODES	(0u)
#define	MODE_CAPTURE_CALIB_IMAGES	(1u)
#define	MODE_CALIBRATE_CAMERA		(2u)
#define	MODE_DETECT_MARKERS			(3u)

/* NAMESPACES */
using namespace std;
using namespace cv;

const char* keys =
"{help h usage ? |   | Show help window}"
"{mode           | 3 | Program modes:	 Generate Aruco codes = 0, "
							"Capture images for calibration = 1, "
							"Calibrate camera = 2, "
							"Detect markers = 3}"
"{ser_com  | COM4 | COM port for serial transmission to KF64 board}"
"{ser_MA   | 1    | Used to smooth x,y, and phi signal (must not be 0!) before being sent to COM port}"
"{ser_T    | 1    | Transmit positions to KF64 board {1=TRUE, 0=FALSE}}"
"{det_FDF  | 50   | Number of frames used for first detection}"
"{det_WES  | 5    | When a marker is lost, window expands by this value [px/s]}"
"{img_O    | 1    | Show 'distorted', original camera image {1=TRUE, 0=FALSE} }"
"{img_U    | 0    | Show image after undistortion {1=TRUE, 0=FALSE}}"
"{img_C    | 1    | Show image portion that is used for detection {1=TRUE,0=FALSE}}"
"{img_ZERO | 1    | Show principal point {1=TRUE,0=FALSE}}"
"{img_CS   | 1    | Show xy-coordinate system {1=TRUE,0=FALSE}}"
"{img_SZ   | 50   | This safety zone value will be added to the cropped image [px]}"
"{cl_WC    | 0    | Print world coordinates to command line}"
"{cl_CSV   | 0    | Print world coordinates to CSV file}"
"{cl_IP    | 0    | Print intrinsic camera parameters}"
"{cl_RM    | 0    | Print rotation matrix for each detected marker}"
"{cl_SER   | 0    | Print serial message to command line {1=TRUE, 0=FALSE}}"
"{d        | 0    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
"DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
"{v        |       | Input from video file, if ommited, input comes from camera }"
"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
"{c        |       | Camera intrinsic parameters. Needed for camera pose }"
"{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
"{dp       |       | File of marker detector parameters }"
"{r        |       | show rejected candidates too }"
"{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
"CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";

int main(int argc, char *argv[])
{
	CommandLineParser parser(argc, argv, keys);
	int error = ERR_OK;
	if ( parser.has("help") )
	{
		parser.printMessage();
		return error;
	}
	// parse values 
	int mode								= parser.get<int>("mode");
		//serial 
	std::string comPort						= parser.get<string>("ser_com");
	const int movingAverageSamples			= parser.get<int>("ser_MA");
	const int transmitSerialData			= parser.get<int>("ser_T");
		//detection
	const int firstDetectionFrames			= parser.get<int>("det_FDF");
	const int windowExpansionSpeed			= parser.get<int>("det_WES");
		//image
	const int showOriginalImage				= parser.get<int>("img_O");
	const int showUndistortedImage			= parser.get<int>("img_U");
	const int showCroppedImage				= parser.get<int>("img_C");
	const int showPrincipalPoint			= parser.get<int>("img_ZERO");
	const int showCoordinateSystemInImage   = parser.get<int>("img_CS");
	const int safetyZone					= parser.get<int>("img_SZ");
		//command line
	const int printWorldCoordinates			= parser.get<int>("cl_WC");
	const int printToCsvFile				= parser.get<int>("cl_CSV");
	const int printIntrinsicParameters		= parser.get<int>("cl_IP");
	const int printRotationMatrix			= parser.get<int>("cl_RM");
	const int printSerialMessage			= parser.get<int>("cl_SER");

	switch (mode)
	{
		case MODE_GENERATE_ARUCO_CODES:
			error = generateAruco();
			break;
		case MODE_CAPTURE_CALIB_IMAGES:
			error = captureSaveCalibImages();
			break;
		case MODE_CALIBRATE_CAMERA:
			error = calibrateCamera();
			break;
		case MODE_DETECT_MARKERS:
			error = detectMarkers(&comPort[0], movingAverageSamples, transmitSerialData, firstDetectionFrames,
									windowExpansionSpeed, showOriginalImage, showUndistortedImage, showCroppedImage,
									showPrincipalPoint, showCoordinateSystemInImage, safetyZone, printWorldCoordinates, 
									printToCsvFile, printIntrinsicParameters, printRotationMatrix, printSerialMessage);
			break;
		default:
			error = ERR_MODE;
			break;
	}
	return error;
}

