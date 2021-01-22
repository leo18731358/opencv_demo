
#include <QtCore/QCoreApplication>
#include <opencv2/opencv.hpp>
#include "import_dll_class.h"
#include "..\ImageAlgorithm\image_process.h"
#include "..\PantographProcessInstance\PantographProcessInstance.h"
#include "..\DealFile\ini_helper.h"
#include <qdebug.h>
#include <qfile.h>

#include <vector>

#define INPUT_IMAGE "input_image"

using namespace cv;
using namespace std;

void readMatImage(Mat &merageImage);


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
	namedWindow(INPUT_IMAGE, WINDOW_NORMAL);

	shared_ptr<ImageProcess> m_pImageProcessPtr = ImportDllClass::GetInstance()->m_pAlgorithmBase->getImageProcessInstance();
	PantographProcessInstance *m_pPantographProcessInstance = ImportDllClass::GetInstance()->m_pPantographProcessInstance;

	Mat merageImage;
	readMatImage(merageImage);

	QString imageWritePath = QCoreApplication::applicationDirPath() + "/original_data/merageImage.jpg";
	imwrite(imageWritePath.toLocal8Bit().toStdString(), merageImage);

	Mat dstImage;
	Rect rectTemp = Rect(Point(1948, 328), Point(2948, 810));
	double precision = 0.45;
	double maxDistance, minDistance;
	m_pPantographProcessInstance->carbonAbrasionDetect(merageImage, rectTemp, precision, dstImage, maxDistance, minDistance);

	QString leftModelImage = QCoreApplication::applicationDirPath() + "/ncc_model/leftModel.jpg";
	Mat rectMatImageLeft = imread(leftModelImage.toLocal8Bit().toStdString(), IMREAD_GRAYSCALE);
	imwrite(leftModelImage.toLocal8Bit().toStdString(), rectMatImageLeft);
	Point localCenterPointLeft;
	double optimalAngleLeft;
	double matchScoreLeft;

	double startTmeTick = getCPUTickCount();
	m_pImageProcessPtr->leo_multiAngleMatchTemplate(merageImage, rectMatImageLeft, -10, 20, 1, 5, 0.5, CV_TM_CCORR_NORMED, localCenterPointLeft, optimalAngleLeft, matchScoreLeft);
	double endTimeTick = getCPUTickCount();

	qDebug() << (endTimeTick - startTmeTick) / getTickFrequency()*1000 << "   "<< optimalAngleLeft << "   " << matchScoreLeft << endl;

	rectangle(dstImage, Point(localCenterPointLeft.x - rectMatImageLeft.cols/2, localCenterPointLeft.y - rectMatImageLeft.rows/2), 
		Point(localCenterPointLeft.x + rectMatImageLeft.cols/2, localCenterPointLeft.y + rectMatImageLeft.rows/2), 
		Scalar(0, 0, 255), 2);


	QString rightModelImage = QCoreApplication::applicationDirPath() + "/ncc_model/rightModel.jpg";
	Mat rectMatImageRight = imread(rightModelImage.toLocal8Bit().toStdString(), IMREAD_GRAYSCALE);
	imwrite(rightModelImage.toLocal8Bit().toStdString(), rectMatImageRight);
	Point localCenterPointRight;
	double optimalAngleRight;
	double matchScoreRight;

	startTmeTick = getCPUTickCount();
	m_pImageProcessPtr->leo_multiAngleMatchTemplate(merageImage, rectMatImageRight, -10, 20, 1, 5, 0.5, CV_TM_CCORR_NORMED, localCenterPointRight, optimalAngleRight, matchScoreRight);
	endTimeTick = getCPUTickCount();

	qDebug() << (endTimeTick - startTmeTick) / getTickFrequency() * 1000 << "   " << optimalAngleRight << "   " << matchScoreRight << endl;

	rectangle(dstImage, Point(localCenterPointRight.x - rectMatImageLeft.cols / 2, localCenterPointRight.y - rectMatImageLeft.rows / 2), 
		Point(localCenterPointRight.x + rectMatImageLeft.cols / 2, localCenterPointRight.y + rectMatImageLeft.rows / 2), 
		Scalar(0, 0, 255), 2);





	QString detWritePath = QCoreApplication::applicationDirPath() + "/original_data/dstImage.jpg";
	imwrite(detWritePath.toLocal8Bit().toStdString(), dstImage);
	imshow(INPUT_IMAGE, dstImage);

	waitKey(0);

    return a.exec();
}


void readMatImage(Mat &merageImage)
{
	IniHelper *iniHelper = ImportDllClass::GetInstance()->m_pIniHelper;
	QString iniFilePath = QCoreApplication::applicationDirPath() + "/original_data/params.ini";
	QString leftMatrixStr = iniHelper->readIniStr("calibration", "leftMatirx", iniFilePath);
	QString rightMatrixStr = iniHelper->readIniStr("calibration", "rightMatirx", iniFilePath);
	QStringList strList;
	strList = leftMatrixStr.split(",");
	double m_dLeftMatrix[3][3];
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			m_dLeftMatrix[i][j] = strList[i * 3 + j].toDouble();
		}
	}
	strList.clear();
	strList = rightMatrixStr.split(",");
	double m_dRightMatrix[3][3];
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			m_dRightMatrix[i][j] = strList[i * 3 + j].toDouble();
		}
	}

	cv::Mat leftMatrix, rightMatrix;
	leftMatrix = Mat(Size(3, 3), CV_64FC1, m_dLeftMatrix);
	rightMatrix = Mat(Size(3, 3), CV_64FC1, m_dRightMatrix);

	QString leftImagePath = QCoreApplication::applicationDirPath() + "/original_data/pantograph_1/s_back_left.jpg";
	QString rightImagePath = QCoreApplication::applicationDirPath() + "/original_data/pantograph_1/s_back_right.jpg";
	Mat leftGrayImage = imread(leftImagePath.toLocal8Bit().toStdString(), IMREAD_GRAYSCALE);
	Mat rightGrayImage = imread(rightImagePath.toLocal8Bit().toStdString(), IMREAD_GRAYSCALE);

	Mat dstLeftGrayImage, dstRightGrayImage;
	warpPerspective(leftGrayImage, dstLeftGrayImage, leftMatrix, dstLeftGrayImage.size(), INTER_LINEAR, BORDER_CONSTANT);
	warpPerspective(rightGrayImage, dstRightGrayImage, rightMatrix, dstRightGrayImage.size(), INTER_LINEAR, BORDER_CONSTANT);

	merageImage = Mat(dstLeftGrayImage.rows, dstLeftGrayImage.cols * 2, CV_8UC1);
	merageImage.setTo(0);
	Mat leftImageRoi = merageImage(Rect(0, 0, dstLeftGrayImage.cols, dstLeftGrayImage.rows));
	Mat rightImageRoi = merageImage(Rect(dstLeftGrayImage.cols, 0, dstLeftGrayImage.cols, dstLeftGrayImage.rows));
	dstLeftGrayImage.copyTo(leftImageRoi);
	dstRightGrayImage.copyTo(rightImageRoi);
}

