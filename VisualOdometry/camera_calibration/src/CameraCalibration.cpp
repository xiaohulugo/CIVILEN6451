#include "CameraCalibration.h"

CameraCalibration::CameraCalibration(void)
{
	int imageNum=0;
	int broadCols=0, broadRows=0;
	int imageCols=0, imageRows=0;
	double reprojectionError=0.0;
}


CameraCalibration::~CameraCalibration(void)
{
}

Point2d projectPoint(Mat_<double> P, Point3d objectPt)
{
	Mat_<double> X = (Mat_<double>(4,1) << objectPt.x,objectPt.y,objectPt.z,1);
	Mat_<double> x = P * X;
	Point2d imgPt(x(0)/x(2),x(1)/x(2));
	return imgPt;
}

void draw(vector<Mat_<double> > ProjectMats, double unitSize, string path, vector<string> filePaths)
{
	int i,j;

	double AxisLen = 4*unitSize, boxSize = 3*unitSize;
	Point3d origPt(0,0,0);
	Point3d XPt(AxisLen,0,0);
	Point3d YPt(0,AxisLen,0);
	Point3d ZPt(0,0,AxisLen);
	Point3d boxPt000, boxPt100, boxPt010, boxPt110;
	Point3d boxPt001, boxPt101, boxPt011, boxPt111;
	boxPt000 = origPt; boxPt001 = boxPt000; boxPt001.z = boxSize;
	boxPt100 = Point3d(boxSize,0,0); boxPt101 = boxPt100; boxPt101.z = boxSize;
	boxPt010 = Point3d(0,boxSize,0); boxPt011 = boxPt010; boxPt011.z = boxSize;
	boxPt110 = Point3d(boxSize,boxSize,0); boxPt111 = boxPt110; boxPt111.z = boxSize;

	int imageNum = filePaths.size();
	for (i = 0; i < imageNum; i ++)
	{
		Mat_<double> P = ProjectMats[i];
		Point2d origPtm, XPtm, YPtm, ZPtm;
		Point2d boxPt000m, boxPt100m, boxPt010m, boxPt110m;
		Point2d boxPt001m, boxPt101m, boxPt011m, boxPt111m;
		string Xlabel = "X", Ylabel = "Y", Zlabel = "Z";
		origPtm = projectPoint(P, origPt);
		XPtm = projectPoint(P, XPt);
		YPtm = projectPoint(P, YPt);
		ZPtm = projectPoint(P, ZPt);
		boxPt000m = origPtm; boxPt001m = projectPoint(P, boxPt001);
		boxPt100m = projectPoint(P, boxPt100); boxPt101m = projectPoint(P, boxPt101);
		boxPt010m = projectPoint(P, boxPt010); boxPt011m = projectPoint(P, boxPt011);
		boxPt110m = projectPoint(P, boxPt110); boxPt111m = projectPoint(P, boxPt111);

		Mat image = imread(filePaths[i]);
		line(image, boxPt000m, boxPt100m, Scalar(255,0,255), 2);
		line(image, boxPt110m, boxPt100m, Scalar(255,0,255), 2);
		line(image, boxPt110m, boxPt010m, Scalar(255,0,255), 2);
		line(image, boxPt010m, boxPt000m, Scalar(255,0,255), 2);

		line(image, boxPt001m, boxPt101m, Scalar(255,0,255), 2);
		line(image, boxPt111m, boxPt101m, Scalar(255,0,255), 2);
		line(image, boxPt111m, boxPt011m, Scalar(255,0,255), 2);
		line(image, boxPt011m, boxPt001m, Scalar(255,0,255), 2);

		line(image, boxPt000m, boxPt001m, Scalar(255,0,255), 2);
		line(image, boxPt110m, boxPt111m, Scalar(255,0,255), 2);
		line(image, boxPt100m, boxPt101m, Scalar(255,0,255), 2);
		line(image, boxPt010m, boxPt011m, Scalar(255,0,255), 2);

		cv::putText(image,Xlabel,XPtm,2,1,Scalar(0,255,255));
		cv::putText(image,Ylabel,YPtm,2,1,Scalar(0,255,255));
		cv::putText(image,Zlabel,ZPtm,2,1,Scalar(0,255,255));
		line(image, origPtm, XPtm, Scalar(0,0,255), 3);
		line(image, origPtm, YPtm, Scalar(0,255,0), 3);
		line(image, origPtm, ZPtm, Scalar(255,0,0), 3);

		char dstImageName[512];
		sprintf(dstImageName,"xiaohu_LU_##%d.jpg",i+1);
		string name = dstImageName;
		name = path + "/" + name;
		imwrite(name, image);
	}
}

bool CameraCalibration::chessboardCornersDetection(std::vector<string> filePaths,std::vector< std::vector<Point2f> > &seqCorners)
{
	int i,j;
	imageNum = filePaths.size();
	Size boardSize = Size(broadCols, broadRows);    

	if (filePaths.size() < 3)
	{
		cout<<"Fail: less than 3 images£¡"<<endl;
		return false;
	}

	cv::Mat curImage;
	cv::Mat curImageGray;
	vector<Point2f> corners;
	for (i = 0; i < imageNum; ++i)
	{   
		corners.clear();

		curImage = imread(filePaths[i]);
		imageCols=curImage.cols;
		imageRows=curImage.rows;
		cvtColor(curImage, curImageGray, CV_BGR2GRAY);
		if (findChessboardCorners( curImageGray, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH ))
		{
			cornerSubPix(curImageGray, corners, Size(11, 11), Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			seqCorners.push_back(corners);

 			Mat imageTemp;
 			curImage.copyTo(imageTemp);
 			drawChessboardCorners(imageTemp, boardSize, Mat(corners), 1);      
 
 			char cornerName[512];
 			sprintf(cornerName,"trace%d.jpg",i+1);
 			string name = cornerName;
 			name = path + "/" + name;
 			imwrite(name, imageTemp);
		}
		else
		{
			cout<<"Fail: can't find chessboard corners!\n";
			return false;
		}
	}
	cout<<"Chessboard Corners Detection Finished\n"<<endl;
	return true;
}

bool CameraCalibration::calibration(std::vector< std::vector<Point2f> > &seqCorners,Mat_<double> &cameraMatrix,vector<double> &distCoeffs
	,vector< vector<Point3f> > &seqObjectPoints,std::vector<std::vector<double> > &seqRotation,std::vector<std::vector<double> > &seqTranslation)
{
	int i,j,m,n;

	Size imageSize = Size(imageCols, imageRows); 
	Size squareSize = Size(28, 28);  

	Point3f point3D;
	vector<Point3f> objectPoints;
	for (m = 0; m < imageNum; m ++)
	{
		objectPoints.clear();
		for (i = 0; i < broadCols; i ++)
		{
			for (j = 0; j < broadRows; j ++)
			{
				point3D.x = i * squareSize.width;
				point3D.y = j * squareSize.height;
				point3D.z = 0;
				objectPoints.push_back(point3D);
			}
		}
		seqObjectPoints.push_back(objectPoints);
	}

	reprojectionError = calibrateCamera(seqObjectPoints, seqCorners, imageSize, cameraMatrix, distCoeffs, seqRotation, seqTranslation);
	cout<<"Calibration Finished\n";
	return true;
}

bool CameraCalibration::drawAxisandBox(Mat_<double> cameraMatrix,std::vector<std::vector<double> > seqRotation,vector< vector<double> > seqTranslation,std::vector<string> filePaths)
{
	int i,j;

	vector<Mat_<double> > projectMats;
	for (i = 0; i < imageNum; i ++)
	{
		Mat_<double> R,T;	
		Rodrigues(seqRotation[i],R);
		T = Mat(Matx31d(seqTranslation[i][0],seqTranslation[i][1],seqTranslation[i][2]));
		Mat_<double> P = cameraMatrix * Mat(Matx34d( R(0,0),R(0,1), R(0,2), T(0),  
			R(1,0),R(1,1), R(1,2), T(1),  
			R(2,0),R(2,1), R(2,2), T(2))); 
		projectMats.push_back(P);
	}

	double unitSize = broadCols;
	draw(projectMats, unitSize, path, filePaths);
	cout<<"Drawing Axis and Box Finished\n";
	return true;
}

bool CameraCalibration::accuracyAssessment(std::vector< std::vector<Point2f> > &seqCorners,Mat_<double> &cameraMatrix,vector<double> &distCoeffs,
	vector< vector<Point3f> > &seqObjectPoints,std::vector<std::vector<double> > &seqRotation,std::vector<std::vector<double> > &seqTranslation)
{
	int i,j;

	std::vector<Point2f> errorList;
	double meanError;

	vector<double> rotation;
	vector<double> translation;
	vector<Point2f> corners;
	for (i = 0; i < imageNum; i ++)
	{
		errorList.clear();
		projectPoints(seqObjectPoints[i], seqRotation[i], seqTranslation[i], cameraMatrix, distCoeffs, errorList);
		meanError = 0;
		corners.clear();
		corners = seqCorners[i];
		for (j = 0; j < corners.size(); j ++)
		{   
			meanError += sqrt((errorList[j].x - corners[j].x)*(errorList[j].x - corners[j].x)+(errorList[j].y - corners[j].y)*(errorList[j].y - corners[j].y));
		}
		rotation.clear();
		translation.clear();
		rotation = seqRotation[i];
		translation = seqTranslation[i];

		char cameraResult[512];
		sprintf(cameraResult,"xiaohu_LU_cam_%d.txt",i+1);
		string name = cameraResult;
		name = path + "/" + name;
		FILE *fp=fopen(name.c_str(),"w");

		//intrinsic
		fprintf(fp,"Camera Matrix\n");
		fprintf(fp,"%lf %lf %lf\n%lf %lf %lf\n%lf %lf %lf\n",cameraMatrix(0,0),cameraMatrix(0,1),cameraMatrix(0,2),
			cameraMatrix(1,0),cameraMatrix(1,1),cameraMatrix(1,2),cameraMatrix(2,0),cameraMatrix(2,1),cameraMatrix(2,2));
		fprintf(fp,"Camera Distortion Parameters");
		for (j = 0; j < distCoeffs.size(); j ++)
		{
			fprintf(fp,"%lf ",distCoeffs[j]);
		}
		fprintf(fp,"\n");

		//extrinsic
		fprintf(fp,"Rotation Angle£");
		fprintf(fp,"%lf %lf %lf\n",rotation[0],rotation[1],rotation[2]);
		fprintf(fp,"Translation");
		fprintf(fp,"%lf %lf %lf\n",translation[0],translation[1],translation[2]);
		fprintf(fp,"Average Projection Error%lf\n",meanError/corners.size());

		fclose(fp);
	}

	cout<<"Accuracy Assessment Finished\n"<<endl;

	return true;
}