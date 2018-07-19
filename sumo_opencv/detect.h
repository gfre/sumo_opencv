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

int detectMarkers(char *serialComPort, const int firstDetectionFrames);
