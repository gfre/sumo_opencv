#pragma once 
#include <Windows.h>

	/* SERIAL SETUP*/
#define SERIAL_COM_PORT						"COM4"					//Select COM port for serial transmission (with KF64 board which appears as "USB Serial Device" in Ports (COM&LPT))

	/* ENABLE FEATURES */
#define GENERATE_ARUCO_CODES				(0x01u)					//Generate and save Aruco Marker
#define CALIBRATION_CAPTURE_SAVE_IMAGES		(0x02u)					//Take and save images for camera calibration
#define CALIBRATION_CALIBRATE_CAMERA		(0x03u)					//Calibrate camera with saved images
#define DETECT_MARKERS						(0x04u)					//Detect markers

	/* SELECT PROGRAM MODE */
#define SUMO_OPENCV_MODE                    (DETECT_MARKERS)

#define NUM_FIRST_DETECTION_COUNTS			(50)					//Number of scancs through first image to make sure camera detects all markers in image
#define EXPAND_WINDOW						(50)					//When not all sumos are detected the cropped window is widened by this amount [px/s]
#define CROPPED_IMAGE_SAFETY_ZONE			(75)					//This value will be added to the cropped image [px]
#define MOVING_AVG_SAMPLES					(1)						//Number of measurements used in moving average before data is send to COM port

#define SHOW_ORIGINAL_IMAGE					(TRUE)					//Show the originally captured image with detected markers and coordinate system.
#define SHOW_UNDISTORTED_IMAGE				(FALSE)					//Show the undistorted image 
#define SHOW_CROPPED_IMAGE					(TRUE)					//Show the cropped image
#define SHOW_FRAME_CENTER					(TRUE)					//Show principal point on image
#define SHOW_FRAME_COORD_SYS				(TRUE)					//Show xy-coordinate system on image

#define MANUAL_REC							(TRUE)					//Enable manual video recordings
#define AUTO_REC							(FALSE)					//Automatically start video capture with transition

#define PRINT_WORLD_COORDS					(FALSE)					//Print 3D World Coordinates (X,Y,Z) to Command line 	
#define PRINT_COORDS_TO_CSV					(FALSE)					//Print 3D World Coordinates (X,Y,Z) to CSV File
#define PRINT_INTR_PARA						(FALSE)					//Print intrinsic camera parameters (Camera matrix and distortion coefficients)
#define PRINT_ROT_MATRIX					(FALSE)					//Print rotation matrix for each detected marker

#define SERIAL_TRANSMIT						(TRUE)					//Enable/Disable Serial Transmission
#define PRINT_SERIAL_MSG_TO_CL				(FALSE)					//Print the Serial Message to Command Line. This is what is being sent to the robots

	/* SAVE COORDINATES TO CSV */
#define CSV_SAVE_ID							(TRUE)					//Set to true if id should be saved as well
#define CSV_USE_COMMA						(TRUE)					//Use a comma as decimal separator 
#define CSV_SEPARATOR						";"
#define CSV_OUTPUT_FILENAME					"datanew.csv"
#define CSV_REMOVE_AT_START					(TRUE)					//Set to true if old csv file should be removed at start 
#define CSV_NO_ID							(-1)
#define CSV_SAVE_TIME						(TRUE)					//Save timestamp for each measurement

	/* CAMERA SETUP */
#define FRAME_WIDTH							(2592)					//Camera image width
#define FRAME_HEIGHT						(2048)					//Camera image height
#define MS_BETWEEN_FRAMES					(1)						// 0 will freeze the program until key is pressed. The larger this value, the slower the data transmission 
#define REC_FPS								(2.5)

	/* ARUCO SETUP */
#define ARUCO_DICT							(aruco::DICT_4X4_50)						//select the predefined Aruco dictionary
#define MARKER_LENGTH						(100)										//Marker length in mm (98 - 150)
#define MAX_NUMBER_OF_MARKERS				(11u)										//How many markers/robots exist
#define NUM_OF_VARIABLES					(3u)											//How many variables per robot (x, y, phi)
#define MAX_MSG_LENGTH						( ((uint8_t)MAX_NUMBER_OF_MARKERS%4u) ? ((((uint8_t)MAX_NUMBER_OF_MARKERS / 4u)+1u)*4u)*(NUM_OF_VARIABLES) : ((MAX_NUMBER_OF_MARKERS)*(NUM_OF_VARIABLES)) )
#define CALIB_FILE_NAME						"camera_parameters/1_k1k2_fix_fl_enabled.xml" //This is the file where opencv takes the distortion coefficients and the camera matrix from

	/* Corner Refinement */
#define CR_ENABLE							(true)
#define CR_WIN_SIZE							(3)
#define CR_MAX_ITERATIONS					(50)
#define CR_MIN_ACCURACY						(0.1)

	/* ChAruco Camera Calibration */
#define CHARUCO_USE_INTRINSIC_GUESS			(TRUE)	//Has to be true if CHARUCO_FLAGS contains CV_CALIB_USE_INTRINSIC_GUESS
/* 
distCoeffs looks as follows:  [k1 k2 p1 p2 [k3 [k4 k5 k6 [s1 s2 s3 s4 [taux tauy]]]]
k's denote radial distortion parameters,
p's denote tangential distortion parameters,
s's denote thin prism distortion parameters,
tau's denote tilted sensor parameters 

CV_CALIB_RATIONAL_MODEL	   enables parameters k4-k6
CV_CALIB_THIN_PRISM_MODEL  enables parameters s1-s4
CV_CALIB_TILTED_MODEL      enables parameters taux, tauy

CV_CALIB_FIX_TANGENT_DIST  sets p1 and p2 to zero 
CV_CALIB_FIX_K3            sets k3 to zero 
CV_CALIB_FIX_S1_S2_S3_S4   sets s1-s4 to zero
CV_CALIB_FIX_TAUX_TAUY	   sets taux and tauy to zero
*/
#define CHARUCO_FLAGS						(CV_CALIB_FIX_FOCAL_LENGTH | CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_K3 | CV_CALIB_RATIONAL_MODEL)		
#define CHARUCO_CAM_ID						(0)								//Select which camera to calibrate
#define CHARUCO_NUM_SQUARES_X				(4)
#define CHARUCO_NUM_SQUARES_Y				(6)
#define CHARUCO_SQUARE_LENGTH				(397)							//Charuco Board square length in pixel
#define CHARUCO_MARKER_LENGTH				(283)							//Charuco Board marker length in pixel
#define CHARUCO_SQUARE_LENGTH_M				(0.123) //(0.12165)				//Charuco Board square length in meter
#define CHARUCO_MARKER_LENGTH_M				(0.094) //(0.09405)				//Charuco Board marker length in meter
#define CHARUCO_FOCAL_LENGTH_EST			(1672) //(1667)							//Intrinsic guess of focal length
#define CHARUCO_REFIND_STRATEGY				(FALSE)							//Use refind strategy to find markers based on previously found markers
#define CHARUCO_FILENAME_CALIB_CAMERA		"camera_parameters/7_k1k2p1p2k4k5k6_fix_fl_enabled.xml"	//Name of output file of generated calibration parameters
#define CHARUCO_FILENAME_CALIB_IMAGES		"images/calibImage_"			//This is the path and file base name which will be concatenated with an index for each image
#define CHARUCO_FILENAME_CALIB_IMAGES_SUFFIX ".png"							//This is the suffix for the taken images
#define CHARUCO_ASPECT_RATIO				(1)
#define CHARUCO_PRINT_FINAL					(TRUE)							//Print calibration errors and loaded images
#define CHARUCO_SHOW_CHESSBOARD_CORNERS		(FALSE)							//Show detected Charuco Corners on Image

	/* ERROR CODES */
#define ERR_OK								(0x00u)	
#define ERR_INV_PARAM_FILE					(0x01u)
#define ERR_NO_ORIGIN						(0x02u)
#define ERR_SERIAL_CONFIG					(0x03u)
#define ERR_FILE_NOT_OPEN					(0x04u)
#define ERR_CAMERA_OPEN						(0x05u)
#define ERR_CAMERA_SET_RESOLUTION			(0x06u)
#define ERR_INPUTVIDEO_GRAB					(0x07u)
#define ERR_INPUTVIDEO_RETRIEVE				(0x08u)
#define ERR_USER_ESCAPE						(0x09u)
#define ERR_IMAGE_WRITE						(0x10u)
#define ERR_IMAGE_READ						(0x11u)

#define CHARUCO_ERR_NOT_ENOUGH_FRAMES		(5u)
#define CHARUCO_ERR_NOT_ENOUGH_CORNERS		(6u)
#define CHARUCO_ERR_CALIB_FILE				(7u)

#define ERR_STR_NO_ORIGIN					"No Origin Marker detected"
#define ERR_STR_NO_MARKER					"No Marker detected"

/* Commands that are sent to sumos */
#define VAR_INVALID							(0x8000u)				//Send this instead of coordinates if marker was not detected
#define VAR_SWRESET							(0x7F00u)				//Invoke software reset
#define VAR_IDLE							(0x7F11u)				//Go to IDLE application state 
#define VAR_READY							(0x7F22u)				//Go to NORMAL application state
#define VAR_TRANS							(0x7F33u)				//Start next transition