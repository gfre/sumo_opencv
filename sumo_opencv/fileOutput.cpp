/* PROJECT INCLUDES */
#include "config.h"
/* SYSTEM INCLUDES */
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <string>
#include <locale>
/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"

using namespace std;

/* Use comma as decimal separator
http://www.cplusplus.com/forum/beginner/5657/
*/
class CommaSeparator : public numpunct<char> // class for decimal numbers with comma
{
protected: char do_decimal_point() const { return ','; } // override the function that gives the decimal separator
};


bool fileExists(const string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

int removeCSVFile()
{
	int errorCode = ERR_OK;

	string fileName = CSV_OUTPUT_FILENAME;
	remove(fileName.c_str());

	return errorCode;
}


int writeCoordsToCSV(cv::Mat coordinates, int id=CSV_NO_ID)
{
	int errorCode = ERR_OK;

	ofstream outputFile;

	// Create File
	outputFile.open(CSV_OUTPUT_FILENAME, ios::out | ios::app);

	if (!outputFile.is_open())
	{
		errorCode = ERR_FILE_NOT_OPEN;
	}
	else {

#if CSV_USE_COMMA
		//Use comma as decimal divider
		locale comma_locale(locale(), new CommaSeparator);
		outputFile.imbue(comma_locale);
#endif
		if (CSV_NO_ID != id & CSV_SAVE_ID)
		{
			outputFile << id;
			outputFile << CSV_SEPARATOR;
		}

		outputFile << coordinates.at<double>(0, 0);
		outputFile << CSV_SEPARATOR;
		outputFile << coordinates.at<double>(1, 0);
		outputFile << "\n";

		outputFile.close();

	}

	return errorCode;
}