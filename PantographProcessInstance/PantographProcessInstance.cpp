
#include "PantographProcessInstance.h"
#include "import_dll_class.h"
#include "..\ImageAlgorithm\image_process.h"

PantographProcessInstance::PantographProcessInstance()
{

}

PantographProcessInstance::~PantographProcessInstance()
{

}

bool PantographProcessInstance::carbonAbrasionDetect(cv::Mat srcImage, cv::Rect rect, double pixelPrecision, cv::Mat &dstImage, double &maxThickness, double &minThickness)
{
	if (!srcImage.data)
	{
		return false;
	}

	cv::Mat cropImage = srcImage(rect);
	int cropImageChannels = cropImage.channels();
	if (1 != cropImageChannels)
	{
		cv::cvtColor(cropImage, cropImage, CV_BGR2GRAY);
	}
	
	std::shared_ptr<ImageProcess> m_pImageProcessPtr = ImportDllClass::GetInstance()->m_pAlgorithmBase->getImageProcessInstance();
	cv::Mat histoGraph, histoGraphData;
	m_pImageProcessPtr->leo_getGrayImageHistData(cropImage, cv::Mat(), histoGraph, histoGraphData);

	int grayValue = 0;
	m_pImageProcessPtr->leo_getPercentHistoValue(histoGraphData, 0.5, grayValue);

	cv::Mat dstThreshold;
	m_pImageProcessPtr->leo_threshold(cropImage, dstThreshold, grayValue, 255);

	cv::Mat kerner1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(500, 2));
	morphologyEx(dstThreshold, dstThreshold, cv::MORPH_OPEN, kerner1);

	std::vector<std::vector<cv::Point>> dstContours;
	findContours(dstThreshold, dstContours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	std::vector<cv::Moments> contourMo(dstContours.size());
	for (size_t i = 0; i < dstContours.size(); i++)
	{
		contourMo[i] = moments(dstContours[i], false);
	}

	std::vector<cv::Point2f>contourCenterPoint(dstContours.size());
	for (size_t i = 0; i < dstContours.size(); i++)
	{
		contourCenterPoint[i] = cv::Point2f(static_cast<float>(contourMo[i].m10 / contourMo[i].m00), static_cast<float>(contourMo[i].m01 / contourMo[i].m00));
	}

	int carbonIndex = 0;
	cv::Point2f carbonPoint(9999999.99, 999999.99);
	for (size_t i = 0; i < contourCenterPoint.size(); i++)
	{
		if (contourCenterPoint[i].y < carbonPoint.y)
		{
			carbonPoint.x = contourCenterPoint[i].x;
			carbonPoint.y = contourCenterPoint[i].y;
			carbonIndex = i;
		}
	}

	std::vector<std::vector<cv::Point>> carbonRegion(1, dstContours[carbonIndex]);
	cv::Mat background;
	cropImage.copyTo(background);
	background.setTo(0);
	drawContours(background, carbonRegion, 0, cv::Scalar::all(255), -1);

	cv:: Mat cannyEdge;
	cv::Canny(background, cannyEdge, 128, 200);

	std::vector<std::vector<cv::Point>> lineContours;
	findContours(cannyEdge, lineContours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	if (lineContours.size() != 2)   //lineContours的大小必须是2
	{
		return false;
	}

	int sumLine[2];
	for (size_t i = 0; i < lineContours.size(); i++)
	{
		for (size_t j = 0; j < lineContours[i].size(); j++)
		{
			sumLine[i] += lineContours[i][j].y;
		}
	}
	int lineUpContourIndex = 0;
	int lineDownContourIndex = 1;
	if (sumLine[0] > sumLine[1])
	{
		lineDownContourIndex = 0;
		lineUpContourIndex = 1;
	}

	cv::Vec4f linePoint;
	fitLine(lineContours[lineDownContourIndex], linePoint, cv::DIST_L2, 0, 1e-2, 1e-2);

	cv::Point linePoint1, linePoint2;
	m_pImageProcessPtr->convert_lineVec4f_to_point(linePoint, linePoint1, linePoint2);

	cv::Mat colorCropImage;
	cv::cvtColor(cropImage, colorCropImage, CV_GRAY2BGR);
	drawContours(colorCropImage, lineContours, 1, cv::Scalar(0, 0, 255), 2);
	line(colorCropImage, linePoint1, linePoint2, cv::Scalar(0, 0, 255), 2);

	std::vector<double> p2lDistance;
	for (size_t i = 0; i < lineContours[lineUpContourIndex].size(); i++)
	{
		double p2lDistanceValue;
		m_pImageProcessPtr->leo_getDistance_p2l(lineContours[lineUpContourIndex][i], linePoint1, linePoint2, p2lDistanceValue);
		p2lDistance.push_back(p2lDistanceValue);
	}
	std::vector<double> sortDistanceValue(p2lDistance);
	std::sort(sortDistanceValue.begin(), sortDistanceValue.end(), std::less<double>());

	double minDistance, maxDistance;
	minDistance = sortDistanceValue[0];
	maxDistance = sortDistanceValue[sortDistanceValue.size() - 1];

	std::vector<double>::iterator minResultIt = find(p2lDistance.begin(), p2lDistance.end(), minDistance);
	std::vector<double>::iterator maxResultIt = find(p2lDistance.begin(), p2lDistance.end(), maxDistance);
	int minIndex = distance(p2lDistance.begin(), minResultIt);
	int maxIndex = distance(p2lDistance.begin(), maxResultIt);

	int offsetMin, offsetMax;
	if (lineContours[1][minIndex].x < lineContours[1][maxIndex].x)
	{
		offsetMin = -80;
		offsetMax = 80;
	}
	else
	{
		offsetMin = 80;
		offsetMax = -80;
	}

	if (1 != srcImage.channels())
	{
		srcImage.copyTo(dstImage);
	}
	else
	{
		cv::cvtColor(srcImage, dstImage, CV_GRAY2BGR);
	}

	cv::Mat dstRoiImage = dstImage(rect);
	colorCropImage.copyTo(dstRoiImage);
	arrowedLine(dstImage, cv::Point(lineContours[lineUpContourIndex][minIndex].x + rect.x + offsetMin, lineContours[lineUpContourIndex][minIndex].y - 80 + rect.y),
		cv::Point(lineContours[lineUpContourIndex][minIndex].x + rect.x, lineContours[lineUpContourIndex][minIndex].y + rect.y), cv::Scalar(0, 255, 0), 2);
	arrowedLine(dstImage, cv::Point(lineContours[lineUpContourIndex][maxIndex].x + rect.x + offsetMax, lineContours[lineUpContourIndex][maxIndex].y - 80 + rect.y),
		cv::Point(lineContours[lineUpContourIndex][maxIndex].x + rect.x, lineContours[lineUpContourIndex][maxIndex].y + rect.y), cv::Scalar(0, 255, 0), 2);

	std::stringstream minDistanceSS, maxDistanceSS;
	minThickness = p2lDistance[minIndex] * pixelPrecision;
	minDistanceSS << std::fixed << std::setprecision(2) << minThickness;
	std::string minDistanceValue = minDistanceSS.str() + "mm";
	maxThickness = p2lDistance[maxIndex] * pixelPrecision;
	maxDistanceSS << std::fixed << std::setprecision(2) << maxThickness;	
	std::string maxDistanceValue = maxDistanceSS.str() + "mm";

	putText(dstImage, minDistanceValue, cv::Point(lineContours[lineUpContourIndex][minIndex].x + rect.x + offsetMin, lineContours[lineUpContourIndex][minIndex].y - 90 + rect.y), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 0), 2);
	putText(dstImage, maxDistanceValue, cv::Point(lineContours[lineUpContourIndex][maxIndex].x + rect.x + offsetMax, lineContours[lineUpContourIndex][maxIndex].y - 90 + rect.y), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 0), 2);

	return true;
}






PantographProcessInstance* PantographProcessInstance::instance = NULL;
PANTOGRAPHPROCESSINSTANCE_EXPORT PantographProcessInstance* getPantographProcessInstance()
{
	if (NULL == PantographProcessInstance::instance)
	{
		PantographProcessInstance::instance = new PantographProcessInstance();
	}
	return PantographProcessInstance::instance;
}

