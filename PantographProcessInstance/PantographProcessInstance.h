
#pragma once
#include <opencv2\opencv.hpp>

#include "pantographprocessinstance_global.h"

class PANTOGRAPHPROCESSINSTANCE_EXPORT PantographProcessInstance
{
public:
    PantographProcessInstance();
	~PantographProcessInstance();


public:
	virtual bool carbonAbrasionDetect(cv::Mat srcImage, cv::Rect rect, double pixelPrecision, cv::Mat &dstImage, double &maxThickness, double &minThickness);



public:
	static PantographProcessInstance *instance;
};

extern "C"
{
	PANTOGRAPHPROCESSINSTANCE_EXPORT PantographProcessInstance* getPantographProcessInstance();
}
