/* SYSTEM INCLUDES */
#include <Windows.h>
#include <stdint.h>
#include <iostream>
#include <stdio.h>
/* PROJECT INCLUDES */
#include "config.h"
#include "opencv2\opencv.hpp"

using namespace std;

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

#if SERIAL_READ_DATA
	int16_t szBuf[] = { 0 };
	DWORD dwBytesRead = 0;
	ReadFile(hComm, szBuf, sizeof(szBuf) - 1, &dwBytesRead, NULL);
#endif


	CloseHandle(hComm);
	return errorCode;


}

int composeSerialMessage(uint16_t *message_, map<int, cv::Mat> *marker, map<int, cv::Mat> *cornerBottomLeft)
{

	int errorCode = ERR_OK;

	for (int i = 0; i < MAX_NUMBER_OF_MARKERS; i++)
	{
		if (!((*marker)[i]).empty())
		{
			//Marker with ID = i was detected

			uint16_t phi = (uint16_t)((*marker)[i].at<double>(0, 0) - (*cornerBottomLeft)[i].at<double>(0, 0));

			message_[3 * i] = (uint16_t)((*marker)[i].at<double>(0, 0)); // x - value
			message_[3 * i + 1] = (uint16_t)((*marker)[i].at<double>(1, 0)); // y - value
			message_[3 * i + 2] = phi; // phi - value
		}
		else
		{
			//Marker with ID = i was NOT detected -> Transmit VAR_INVALID

			message_[3 * i] = VAR_INVALID; // x - value
			message_[3 * i + 1] = VAR_INVALID; // y - value
			message_[3 * i + 2] = VAR_INVALID; // phi; // phi - value
		}
	}

	return errorCode;
}

