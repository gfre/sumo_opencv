/* OPENCV INCLUDE */
#include "opencv2\opencv.hpp"
/* PROJECT INCLUDES */
#include "config.h"

using namespace cv;
using namespace std;

//Get World Coordinates (X,Y,Z) from Image Coordinates (u,v,1)
int getWorldCoordinates(Point2f uv, Mat *xyz, Mat invCamMatrix, Mat invRotMatrix, Vec3d tvec, double zConst) {

	double s;
	Mat uvPoint, t, tmp, tmp2;

	uvPoint = Mat::ones(3, 1, DataType<double>::type);
	uvPoint.at<double>(0, 0) = uv.x;
	uvPoint.at<double>(1, 0) = uv.y;

	//Copy translation vector tvec to Mat object t
	t = Mat::ones(3, 1, DataType<double>::type);
	t.at<double>(0, 0) = tvec[0];
	t.at<double>(1, 0) = tvec[1];
	t.at<double>(2, 0) = tvec[2];

#if ROTATE_FIRST

	//calculate temp values to solve eq for scaling factor s
	tmp = invRotMatrix * invCamMatrix * uvPoint;
	tmp2 = invRotMatrix * t;

	s = (zConst + tmp2.at<double>(2, 0)) / tmp.at<double>(2, 0);

	*xyz = invRotMatrix * (s * invCamMatrix * uvPoint - t);
	cout << "uv: " << uv.x << "  " << uv.y << endl;
	cout << "t: " << t << endl;
	cout << "RotMatrix: " << invRotMatrix <<  endl;
	cout << (invRotMatrix*(s*invCamMatrix * uvPoint)-t) << endl << endl << endl;
#else

	tmp = invRotMatrix * invCamMatrix * uvPoint;
	s = (zConst + t.at<double>(2, 0)) / tmp.at<double>(2, 0);
	*xyz = (invRotMatrix * s * invCamMatrix * uvPoint) - t;
#endif


	return ERR_OK;

}

