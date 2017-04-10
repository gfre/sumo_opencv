#pragma once

typedef enum sendState_e
{
	 STATE_IDLE = 0x00
	,STATE_RUN
	,STATE_ERROR

} sendState_t;

int detectMarkers();
