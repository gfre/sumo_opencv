#pragma once

/* SETUP */
#define SERIAL_TRANSMIT			(FALSE)										//Enable/Disable Serial Transmission
#define RECALCULATE_HOMOGRAPHY	(FALSE)										//Set to 1 if camera settings/position changed or Homography Matrix needs to be calculated again
#define GENERATE_ARUCO_CODES	(FALSE)										//Generate and save Aruco Marker
#define ROTATE_FIRST			(TRUE)										//Camera Pinhole Model: Roate First translate second or other way round
#define SHIFT_POINT_TO_CENTER	(TRUE)										//Use center Point of Aruco for localization instead of Top Left Corner
#define USE_STITCHER			(FALSE)										//Use 2 camera feeds and stitch images together
#define UNDISTORT_IMAGE			(FALSE)	//!!Warning: if true, image may be undistorted twice for pose estimation
#define PRINT_SERIAL_MSG_TO_CL	(FALSE)										//Print the Message to command Line
#define SHOW_FINAL_IMAGE		(TRUE)										//Show the Live Image with detected Markers and Coordinate System. May slow down the program 

#define FRAME_WIDTH				(1920)										// Camera image width
#define FRAME_HEIGHT			(1080)										// Camera image height
#define MS_BETWEEN_FRAMES		(10)										// wait 10ms before grabbing new frame, may impact performance

#define CALIB_FILE_NAME			"calibration.xml"

#define MARKER_LENGTH			(100)										//Marker length in mm
#define MAX_NUMBER_OF_MARKERS	(4)											//How many markers/robots exist
#define NUM_OF_VARIABLES		(3)											//How many variables per robot (x, y, phi)
#define MAX_MSG_LENGTH ((MAX_NUMBER_OF_MARKERS)*(NUM_OF_VARIABLES))
#define ORIGIN_MARKER_ID		(24)										//Select which marker acts as the origin of world coordinate system
#define MIN_HESSIAN				(500)										//Minimum Hessian threshold for Surf Algorithm
#define Z_CONST					(0)											//Distance from Camera to Marker Plane in mm

#define HOMOGRAPHY_M			0.9568005531058007, 0.001879261708537507, 488.6830366627206, -0.02376243394348695, 0.9520233532133866, 17.3904366427672, -5.302341910721238e-06, -2.612583953997515e-05, 1

#define ERR_OK					(0)							
#define VAR_INVALID				(0xFFEEu)									//Send this instead of coordinates if marker was not detected

#define M_PI					(3.14159265358979323846)					//Pi
