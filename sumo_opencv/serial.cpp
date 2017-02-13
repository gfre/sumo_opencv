/* SYSTEM INCLUDES */
#include <Windows.h>
#include <stdint.h>
#include <iostream>
#include <stdio.h>

using namespace std;

//Send data via serial communication port
int sendSerial(char *serialPort, int maxBufSize, uint16_t *sendBuf, uint8_t byteSize) {
	/*
	https://msdn.microsoft.com/en-us/library/ff802693.aspx
	http://www.dreamincode.net/forums/topic/322031-serial-communication-with-c-programming/
	*/

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
		return 1;
	}

	if (!SetCommTimeouts(hComm, &timeouts)) {
		cout << "Failed with error: " << GetLastError() << endl;
		return 2;
	}

	hStatus = GetCommState(hComm, &dcb);

	if (!hStatus) {
		cout << "GetCommState failed with error: " << GetLastError() << endl;
		return 3;
	}

	//Setup Serial Connection
	dcb.BaudRate = CBR_57600;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;

	hStatus = SetCommState(hComm, &dcb);

	if (!hStatus) {
		cout << "SetCommState failed with error: " << GetLastError() << endl;
		return 4;
	}
	else {
		//cout << "Serial Port " << serialPort << " successfully configured" << endl;
	}


	//Write to serial port
	if (sizeof(sendBuf) > maxBufSize) {
		cout << "Message is too long." << endl;
		return 5;
	}

	if (WriteFile(hComm, sendBuf, byteSize, &dwBytesWrite, NULL)) {
		//cout << "Message: \"" << sendBuf << "\" successfully transmitted" << endl;
	}
	//Uncomment to Read Data from Serial Port
	/*
	int16_t szBuf[] = { 0 };
	DWORD dwBytesRead = 0;
	ReadFile(hComm, szBuf, sizeof(szBuf) - 1, &dwBytesRead, NULL);
	*/


	CloseHandle(hComm);
	return 0;


}
