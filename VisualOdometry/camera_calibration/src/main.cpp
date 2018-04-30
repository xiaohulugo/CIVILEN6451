#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include "CameraCalibration.h"

#include <fstream> 
#include <filesystem>

using namespace std;

std::vector<string> get_filelist(string foldName)
{
	vector<string> extendName;
	extendName.push_back("jpg");
	extendName.push_back("JPG");
	extendName.push_back("bmp");
	extendName.push_back("png");
	extendName.push_back("gif");

	std::vector<string> image_paths;
	for (size_t i = 0; i < extendName.size(); i++)
	{
		std::vector<cv::String> image_paths_temp;
		cv::glob(foldName + "/*." + extendName[i], image_paths_temp, false);

		for (size_t j = 0; j < image_paths_temp.size(); j++)
		{
			image_paths.emplace_back(std::string(image_paths_temp[j].c_str()));
		}
	}

	return image_paths;
}

void main()
{   
	string folderPath = "C:\\DeveloperCenter\\Code\\calibration\\data"; 
	std::vector<string> filePaths;
	filePaths = get_filelist(folderPath);

	if (filePaths.size() < 3)
	{
		cout<<"no valide images!"<<endl;
		return;
	}

	CameraCalibration calibrater;
	calibrater.path=folderPath;
	calibrater.broadCols=7;
	calibrater.broadRows=9;
	//cout<<"Input the cols and rows of the chessboard: ( default value is 19 & 19 )"<<endl;
	//cin>>calibrater.broadCols>>calibrater.broadRows;

	//chessboard corners detection
	std::vector< std::vector<Point2f> > seqCorners;
	calibrater.chessboardCornersDetection(filePaths,seqCorners);

	//calibration
	Mat_<double> cameraMatrix;
	std::vector<double> distCoeffs;
	std::vector<std::vector<Point3f> > seqObjectPoints;
	std::vector<std::vector<double> > seqRotation;
	std::vector<std::vector<double> > seqTranslation;
	calibrater.calibration(seqCorners,cameraMatrix,distCoeffs,seqObjectPoints,seqRotation,seqTranslation);
	cout << cameraMatrix << endl;
	//draw axis and box
	calibrater.drawAxisandBox(cameraMatrix,seqRotation,seqTranslation,filePaths);

	//accuracy assessment
	calibrater.accuracyAssessment(seqCorners,cameraMatrix,distCoeffs,seqObjectPoints,seqRotation,seqTranslation);
}

