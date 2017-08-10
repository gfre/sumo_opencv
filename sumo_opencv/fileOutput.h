#pragma once
/* PROJECT INCLUDE */
#include "config.h"

/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"
#include <time.h>
#include <iostream>
#include <fstream>

int removeCSVFile();

int writeCoordsToCSV(cv::Mat coordinates, int id);

int writeMsgToCSV(int16_t *message, int msgLength, long double timestamp, std::ofstream *outputFile);