#pragma once
/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
/* SYSTEM INCLUDES */
#include "config.h"


int removeCSVFile();

int writeCoordsToCSV(cv::Mat coordinates, int id = CSV_NO_ID);
