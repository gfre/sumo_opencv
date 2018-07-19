#define _USE_MATH_DEFINES
/* SYSTEM INCLUDES */
#include <windows.h>
#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <map>
#include <vector>
/* PROJECT INCLUDES */
#include "config.h"
#include "opencv2\opencv.hpp"

using namespace std;

/* GLOBAL VARIABLES */
static map <int, int> mvgAvgIndex;
static map <int, int> mvgAvgIter;
static map<int, map<int, cv::Mat> > markerMvgAvg;
static map<int, map<int, double> >  phiMvgAvg;

//Send data via serial communication port
int sendSerial(char *serialPort, int maxBufSize, uint16_t *sendBuf, uint8_t byteSize)
{
	/*
	https://msdn.microsoft.com/en-us/library/ff802693.aspx
	http://www.dreamincode.net/forums/topic/322031-serial-communication-with-c-programming/
	*/

	int errorCode = 0u;

	/*Serial Setup*/
	HANDLE hComm;
	BOOL hStatus;
	DCB dcb = { 0 };
	COMMTIMEOUTS timeouts = { 0 };
	DWORD dwBytesWrite = 0;

	/*Timeouts*/
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	/*Open Comm Port
	https://msdn.microsoft.com/en-us/library/windows/desktop/aa363858(v=vs.85).aspx
	HANDLE WINAPI CreateFile(
	_In_     LPCTSTR               lpFileName,
	_In_     DWORD                 dwDesiredAccess,
	_In_     DWORD                 dwShareMode,
	_In_opt_ LPSECURITY_ATTRIBUTES lpSecurityAttributes,
	_In_     DWORD                 dwCreationDisposition,
	_In_     DWORD                 dwFlagsAndAttributes,
	_In_opt_ HANDLE                hTemplateFile
	);

	*/
	hComm = CreateFileA(serialPort,
		GENERIC_READ | GENERIC_WRITE,
		0,								//Must be zero
		0,
		OPEN_EXISTING,					//Must specify the OPEN_EXISTING flag
		FILE_ATTRIBUTE_NORMAL,
		NULL);							//Must be NULL


										/*Setup Com Port*/
	if (hComm == INVALID_HANDLE_VALUE) {
		cout << "Error opening port." << endl << "Failed with error: " << GetLastError() << endl;
		
	}

	if (!SetCommTimeouts(hComm, &timeouts)) {
		cout << "Failed with error: " << GetLastError() << endl;
		
	}

	hStatus = GetCommState(hComm, &dcb);

	if (!hStatus) {
		cout << "GetCommState failed with error: " << GetLastError() << endl;
		
	}

	//Setup Serial Connection
	dcb.BaudRate = CBR_57600;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	hStatus = SetCommState(hComm, &dcb);

	if (!hStatus) {
		cout << "SetCommState failed with error: " << GetLastError() << endl;
		
	}
	else {
		//cout << "Serial Port " << serialPort << " successfully configured" << endl;
	}


	//Write to serial port
	if (sizeof(sendBuf) > maxBufSize) {
		cout << "Message is too long." << endl;
		
	}

	if (WriteFile(hComm, sendBuf, byteSize, &dwBytesWrite, NULL)) {
		//cout << "Message: \"" << sendBuf << "\" successfully transmitted" << endl;
	}


	CloseHandle(hComm);
	return errorCode;


}

int composeSerialMessage(uint16_t *message_, map<int, cv::Mat> &marker, map<int, double> &phi, const int movingAverageSamples)
{
	int errorCode = ERR_OK;

	for (int i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
	{

		if (!(marker[i].empty()))
		{		
				
			if (mvgAvgIndex.empty())
			{
				for (int j = 0; j < MAX_NUMBER_OF_MARKERS; j++)
				{
					mvgAvgIndex[j] = 0;
					mvgAvgIter[j]  = 0;
				}
			}

			/* moving average implementation */
			markerMvgAvg[mvgAvgIndex[i]][i] = marker[i];
			phiMvgAvg[mvgAvgIndex[i]][i] = phi[i];
			
			mvgAvgIndex[i] = (mvgAvgIndex[i] + 1) % movingAverageSamples;
			mvgAvgIter[i]++;

			//Marker with ID = i was detected

			if (movingAverageSamples <= mvgAvgIter[i])
			{
				double xAvg		= 0.0;
				double yAvg		= 0.0;
				double phiAvg	= 0.0;

				for (int j = 0; j < movingAverageSamples; j++)
				{
					double x = markerMvgAvg[j][i].at<double>(0, 0);
					double y = markerMvgAvg[j][i].at<double>(1, 0);

					xAvg += x;
					yAvg += y;
					phiAvg += phiMvgAvg[j][i];
				}

				xAvg	= xAvg / (double)movingAverageSamples;
				yAvg	= yAvg / (double)movingAverageSamples;
				phiAvg	= phiAvg / (double)movingAverageSamples * 180.0 / M_PI;
				
				double phiTemp = -phi[i] + (M_PI);
				if (phiTemp > M_PI) phiTemp = -M_PI + (phiTemp - M_PI);
				double phiDeg = phiTemp * 180 / M_PI;

				message_[3 * i]		= (uint16_t)(-xAvg);				// x - value
				message_[3 * i + 1] = (uint16_t)(yAvg);				// y - value
				message_[3 * i + 2] = (uint16_t)(phiAvg);   //(phi[i] * 180 / M_PI);			// phi - value

			}
			else
			{
				//Not enough samples
				message_[3 * i]		= VAR_INVALID;								// x - value
				message_[3 * i + 1] = VAR_INVALID;								// y - value
				message_[3 * i + 2] = VAR_INVALID;								// phi - val
			}
		}
		else
		{
			//Marker with ID = i was NOT detected -> Transmit VAR_INVALID
			message_[3 * i]		= VAR_INVALID; // x - value
			message_[3 * i + 1]	= VAR_INVALID; // y - value
			message_[3 * i + 2]	= VAR_INVALID; // phi - value
		}

		
	}

	return errorCode;
}

