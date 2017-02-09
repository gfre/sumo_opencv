#pragma once
#include <Windows.h>
#include <stdint.h>

using namespace std;

//Send data via serial communication port
int sendSerial(char *serialPort, int maxBufSize, uint16_t *sendBuf, uint8_t byteSize);