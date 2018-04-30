#pragma once

#include <stdlib.h>
#include <iostream>
#include <Windows.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include "opencv2/nonfree/features2d.hpp" 
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp> 
#include "opencv2/nonfree/nonfree.hpp"  
#include "cv.h"

using namespace cv;
using namespace std;

class CameraCalibration
{
public:
	CameraCalibration(void);
	~CameraCalibration(void);

	bool chessboardCornersDetection(std::vector<string> filePaths,std::vector< std::vector<Point2f> > &seqCorners);
	bool calibration(std::vector< std::vector<Point2f> > &seqCorners,Mat_<double> &cameraMatrix,vector<double> &distCoeffs
		,vector< vector<Point3f> > &seqObjectPoints,std::vector<std::vector<double> > &seqRotation,std::vector<std::vector<double> > &seqTranslation);
	bool drawAxisandBox(Mat_<double> cameraMatrix,std::vector<std::vector<double> > seqRotation,vector< vector<double> > seqTranslation,std::vector<string> filePaths);
	bool accuracyAssessment(std::vector< std::vector<Point2f> > &seqCorners,Mat_<double> &cameraMatrix,vector<double> &distCoeffs,
		vector< vector<Point3f> > &seqObjectPoints,std::vector<std::vector<double> > &seqRotation,std::vector<std::vector<double> > &seqTranslation);
public:
	string path;
	int imageNum;
	int broadCols, broadRows;
	int imageCols, imageRows;
	double reprojectionError;
};
