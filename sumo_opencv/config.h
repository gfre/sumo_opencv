#pragma once 
#include <Windows.h>


	/* ENABLE FEATURES */
#define GENERATE_ARUCO_CODES				(0x01u)										//Generate and save Aruco Marker
#define CALIBRATE_CAMERA					(0x02u)										//Calibrate Camera
#define DETECT_MARKERS						(0x04u)										//Detect Markers

#define SUMO_OPENCV_MODE                    (DETECT_MARKERS)

#define ROTATE_FIRST						(FALSE)										//Camera Pinhole Model: Rotate First translate second or other way round
#define SHIFT_POINT_TO_CENTER				(TRUE)										//Use center Point of Aruco for localization instead of Top Left Corner
#define USE_STITCHER						(FALSE)										//Use 2 camera feeds and stitch images together
#define UNDISTORT_IMAGE						(FALSE)	//!!Warning: if true, image may be undistorted twice for pose estimation
#define SHOW_FINAL_IMAGE					(TRUE)										//Show the Live Image with detected Markers and Coordinate System. May slow down the program 
#define ENABLE_REC							(TRUE)
#define START_REC_WITH_TRANSITION			(TRUE)

#define PRINT_WOLRD_COORDS					(FALSE)										//Print 3D World Coordinates (X,Y,Z) to Command line 	
#define PRINT_ORIGIN_COORDS					(FALSE)										//Print 3D World Coordinates (X,Y,Z) of Origin Marker to Command line 
#define PRINT_UV_COORDS						(FALSE)										//Print 2D Image Coordinates (u,v) to Command line
#define PRINT_COORDS_TO_CSV					(TRUE)										//Print 3D World Coordinates (X,Y,Z) to CSV File
#define USE_REL_COORDS						(FALSE)										//TRUE: Use coordinates relative to ORGIN_MARKER; FALSE: use coordinates relative to principal point
#define PRINT_INTR_PARA						(FALSE)										//Print intrinsic camera parameters
#define SHOW_FRAME_CENTER					(TRUE)										//Show principal point on image
#define SHOW_FRAME_COORD_SYS				(TRUE)										//Show uv-coordinate system on image

#define RECALCULATE_HOMOGRAPHY				(FALSE)										//Set to 1 if camera settings/position changed or Homography Matrix needs to be calculated again

#define SERIAL_TRANSMIT						(TRUE)										//Enable/Disable Serial Transmission
#define SERIAL_READ_DATA					(TRUE)										//READ Data from Serial
#define PRINT_SERIAL_MSG_TO_CL				(TRUE)										//Print the Serial Message to Command Line

	/* SAVE COORDINATES TO CSV */
#define CSV_SAVE_ID							(TRUE)										//Set to true if id should be saved as well
#define CSV_USE_COMMA						(TRUE)										//Use a comma as decimal separator 
#define CSV_SEPARATOR						";"
#define CSV_OUTPUT_FILENAME					"datanew.csv"
#define CSV_REMOVE_AT_START					(TRUE)										//Set to true if old csv file should be removed at start 
#define CSV_NO_ID							(-1)
#define CSV_SAVE_TIME						(TRUE)										//Save timestamp for each measurement

	/* CAMERA SETUP */
#define FIRST_CAM_ID						(1)											//ID of first camera
#define SEC_CAM_ID							(2)											//ID of second camera
#define FRAME_WIDTH							(2592)	//(1900)										//Camera image width
#define FRAME_HEIGHT						(2048)  //(1900)										//Camera image height
#define MS_BETWEEN_FRAMES					(5)										//wait 10ms before grabbing new frame, may impact performance
#define REC_FPS								(2.5)


	/* ARUCO SETUP */
#define ARUCO_DICT							(aruco::DICT_4X4_50)						//select the predefined Aruco dictionary
#define MARKER_LENGTH						(100)										//Marker length in mm (98 - 150)
#define MAX_NUMBER_OF_MARKERS				(12)											//How many markers/robots exist
#define NUM_OF_VARIABLES					(3)											//How many variables per robot (x, y, phi)
#define MAX_MSG_LENGTH ((MAX_NUMBER_OF_MARKERS)*(NUM_OF_VARIABLES))
#define ORIGIN_MARKER_ID					(25)										//Select which marker acts as the origin of world coordinate system
#define CALIB_FILE_NAME						"camera_parameters/calibration_basler.xml"
	//Corner Refinement
#define CR_ENABLE							(true)
#define CR_WIN_SIZE							(3)
#define CR_MAX_ITERATIONS					(50)
#define CR_MIN_ACCURACY						(0.1)

	/* SERIAL SETUP*/
#define SERIAL_COM_PORT						"COM5"										//Select com port for serial transmission

	/* ChAruco Camera Calibration */
#define CHARUCO_USE_INTRINSIC_GUESS			(TRUE)										//Has to be true if CHARUCO_FLAGS contains CV_CALIB_USE_INTRINSIC_GUESS
#define CHARUCO_FLAGS						(CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_RATIONAL_MODEL | CV_CALIB_FIX_ASPECT_RATIO)		//(CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_RATIONAL_MODEL | CV_CALIB_THIN_PRISM_MODEL | CV_CALIB_TILTED_MODEL)
#define CHARUCO_CAM_ID						(1)											//Select which camera to calibrate
#define CHARUCO_NUM_SQUARES_X				(4)
#define CHARUCO_NUM_SQUARES_Y				(6)
#define CHARUCO_SQUARE_LENGTH				(397)										//Charuco Board square length in pixel
#define CHARUCO_MARKER_LENGTH				(283)										//Charuco Board marker length in pixel
#define CHARUCO_SQUARE_LENGTH_M				(0.12165)									//Charuco Board square length in meter
#define CHARUCO_MARKER_LENGTH_M				(0.09405)									//Charuco Board marker length in meter
#define CHARUCO_FOCAL_LENGTH_EST			(1700)										//Intrinsic guess of focal length
#define CHARUCO_REFIND_STRATEGY				(FALSE)										//Use refind strategy to find markers based on previously found markers
#define CHARUCO_FILENAME_CALIB				"camera_parameters/calibration_basler.xml"  //Name of output file of generated calibration parameters
#define CHARUCO_ASPECT_RATIO				(1)
#define CHARUCO_PRINT_FINAL					(TRUE)
#define CHARUCO_SHOW_CHESSBOARD_CORNERS		(TRUE)										//Show detected Charuco Corners on Image


	/* CONSTANTS */
#define MIN_HESSIAN							(500)										//Minimum Hessian threshold for SURF Algorithm
#define Z_CONST								(2940)										//Distance from Camera to Marker Plane in mm (3150-3250)
#define HOMOGRAPHY_M						0.9481264903687447, -0.0006128029844869035, 812.8437136026427, -0.02500129918606342, 0.9849352009158311, -11.38559156880517, -2.279974889196073e-05, -6.30028688917313e-06, 1
#define MOVING_AVG_SAMPLES					(5)											// Amount of measurements used in Moving Average


	/* ERROR CODES */
#define ERR_OK								(0u)	
#define ERR_INV_PARAM_FILE					(1u)
#define ERR_NO_ORIGIN						(2u)
#define ERR_SERIAL_CONFIG					(3u)
#define ERR_FILE_NOT_OPEN					(4u)

#define CHARUCO_ERR_NOT_ENOUGH_FRAMES		(5u)
#define CHARUCO_ERR_NOT_ENOUGH_CORNERS		(6u)
#define CHARUCO_ERR_CALIB_FILE				(7u)

#define VAR_INVALID							(0xFFFFu)									//Send this instead of coordinates if marker was not detected
#define VAR_START							(0xAAAAu)									//START Message Code
#define VAR_STOP							(0xBBAAu)									//STOP Message Code
#define VAR_READY							(0xCCAAu)									//READY Message Code
#define VAR_TRANS							(0xDDAAu)									//start next TRANSITION MSG Code

#define ERR_STR_NO_ORIGIN					"No Origin Marker detected"
#define ERR_STR_NO_MARKER					"No Marker detected"