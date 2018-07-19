#pragma once

typedef enum sendState_e
{
	 STATE_IDLE = 0x00
	,STATE_READY
	,STATE_RUN
	,STATE_ERROR

} sendState_t;

typedef enum recState_e
{
	NO_REC = 0x00
	, REC
} recState_t;

int detectMarkers(char *serialComPort_, const int movingAverageSamples_, const int transmitSerialData_, 
					const int firstDetectionFrames_, const int windowExpansionSpeed_, const int showOriginalImage_, const int showUndistortedImage_,
					const int showCroppedImage_, const int showPrincipalPoint_, const int showCoordinateSystemInImage_, const int safetyZone_,
					const int printWorldCoordinates_, const int printToCsvFile_, const int printIntrinsicParameters_, const int printRotationMatrix_,
					const int printSerialMessage_);
