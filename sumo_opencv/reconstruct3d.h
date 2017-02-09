#pragma once
#include "opencv2\highgui.hpp"
#include "opencv2\core.hpp"

using namespace cv;

int getWorldCoordinates(Point2f uv, Mat *xyz, Mat invCamMatrix, Mat invRotMatrix, Vec3d tvec, double zConst);