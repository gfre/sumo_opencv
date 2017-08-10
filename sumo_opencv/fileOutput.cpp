/* PROJECT INCLUDES */
#include "config.h"
/* SYSTEM INCLUDES */
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <string>
#include <locale>
#include <time.h>
/* OPENCV INCLUDES */
#include "opencv2\opencv.hpp"

/* MAKROS */
#define NOTHING							" "
#define SAVE_VALID_MARKER(marker_)		 ( (marker_) != -1) ? (marker_) : 0xFFFF )

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

	clock_t elapsedTime;

	ofstream outputFile;

	// Create File
	outputFile.open(CSV_OUTPUT_FILENAME, ios::out | ios::app);

	if (!outputFile.is_open())
	{
		errorCode = ERR_FILE_NOT_OPEN;
	}
	else {

#if CSV_USE_COMMA
		//Use comma as decimal separator
		locale comma_locale(locale(), new CommaSeparator);
		outputFile.imbue(comma_locale);
#endif
		if ((CSV_NO_ID != id) & CSV_SAVE_ID)
		{
			outputFile << id;
			outputFile << CSV_SEPARATOR;
		}

#if CSV_SAVE_TIME
		elapsedTime = clock();
		float seconds = (float)elapsedTime / CLOCKS_PER_SEC;
		outputFile << seconds;
		outputFile << CSV_SEPARATOR;
#endif
		outputFile << coordinates.at<double>(0, 0);
		outputFile << CSV_SEPARATOR;
		outputFile << coordinates.at<double>(1, 0);
		outputFile << "\n";

		outputFile.close();

	}

	return errorCode;
}

int writeMsgToCSV(int16_t *message, int msgLength, long double timestamp, ofstream *outputFile)
{
	int errorCode = ERR_OK;

	if (!outputFile->is_open())
	{
		errorCode = ERR_FILE_NOT_OPEN;
	}
	else {
#if CSV_USE_COMMA
		//Use comma as decimal separator
		locale comma_locale(locale(), new CommaSeparator);
		outputFile->imbue(comma_locale);
#endif

		*outputFile << timestamp << CSV_SEPARATOR << "" << CSV_SEPARATOR;

		for (int i = 0; i < msgLength / NUM_OF_VARIABLES ; i++)
		{
			
			if (message[3 * i] == -1)
			{
				*outputFile << "";
			} 
			else
			{
				*outputFile << message[3 * i];
			}
			
			*outputFile << CSV_SEPARATOR;

			if (message[3 * i + 1] == -1)
			{
				*outputFile << "";
			}
			else
			{
				*outputFile << message[3 * i + 1];
			}
			*outputFile << CSV_SEPARATOR;
			*outputFile << "";
			*outputFile << CSV_SEPARATOR;
		}

		*outputFile << "\n";

		

	}

	return errorCode;
}
