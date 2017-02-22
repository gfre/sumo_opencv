#pragma once
/* SYSTEM INCLUDES */
#include <Windows.h>
#include <stdint.h>
#include <iostream>
/* PROJECT INCLUDES */
#include "config.h"
/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"

using namespace std;

//Send data via serial communication port
int sendSerial(char *serialPort, int maxBufSize, uint16_t *sendBuf, uint8_t byteSize);

int composeSerialMessage(uint16_t *message_, map<int, cv::Mat> &marker, map<int, double> &phi);