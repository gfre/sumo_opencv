#pragma once
/* PROJECT INCLUDE */
#include "config.h"

/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"
#include "opencv2\opencv_modules.hpp"


int removeCSVFile();

int writeCoordsToCSV(cv::Mat coordinates, int id);
