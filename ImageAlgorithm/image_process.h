#pragma once

#include <QObject>
#include "opencv2\opencv.hpp"

enum RoiFeature{ ROI_RECTANGLE, ROI_CIRCLE, ROI_POLYGON};

class ImageProcess : public QObject
{
	Q_OBJECT

public:
	ImageProcess(QObject *parent=Q_NULLPTR);
	~ImageProcess();

#pragma region 外部接口函数区

	virtual bool get_region(cv::Mat srcImage, std::vector<cv::Point> vps, cv::Mat &region);           //获得Region
	virtual bool get_region(cv::Mat srcImage, cv::Point point1, cv::Point point2, cv::Mat &region);   //获得Region
	virtual bool get_region(cv::Mat srcImage, cv::Point point1, float radius, cv::Mat &region);       //获得Region
	virtual bool get_reduce_domain(cv::Mat srcImage, cv::Mat region, cv::Mat &reduceDaomain);         //获得Region的图像

	virtual bool get_horizontal_and_vertical_line(std::vector<cv::Vec4f> lines, std::vector<cv::Point> &upLine,
		std::vector<cv::Point> &downLine, std::vector<cv::Point> &leftLine, std::vector<cv::Point> &rightLine);

	virtual bool convert_lineVec4f_to_point(cv::Vec4f vec4f, cv::Point &point1, cv::Point &point2);
	virtual bool get_cross_point(cv::Point point1, cv::Point point2, cv::Point point3, cv::Point point4,
		cv::Point &outCrossPoint);


	virtual bool leo_threshold(cv::Mat srcImage, cv::Mat &dstImage, int lowValue, int highValue);                                     //对图像进行二值化
	virtual bool leo_getGrayImageHistData(cv::Mat srcImage,cv::Mat mask ,cv::Mat &histoGraph, cv::Mat &histoData);                    //获取灰度直方图数据
	virtual bool leo_getPercentHistoValue(cv::Mat histoGraphData, float percentValue, int &grayValue);                                //获得特定百分比的灰度值

	virtual bool leo_getDistancs_p2p(cv::Point point1, cv::Point point2, double &distance);                                           //求两点之间的距离
	virtual bool leo_getDistance_p2l(cv::Point PointDis, cv::Point pointLstart, cv::Point pointLend, double &distance);               //求点到直线的距离
	virtual bool leo_fillRegion(cv::Mat byteRegion, cv::Mat &dstRegion);
	virtual bool leo_selectShapeMax(cv::Mat srcImage, std::vector<std::vector<cv::Point>> allRegionContour, cv::Mat &maxRegion);

	virtual bool leo_imageRotate(cv::Mat image, double angle, cv::Mat &outImage);
	virtual bool leo_imagePyrDown(cv::Mat image, int numLevels, cv::Mat &outImage);
	virtual bool leo_multiAngleMatchTemplate(cv::Mat srcImage, cv::Mat modelImage, double angleStart, double angleRange, double angleStep, int numLevels, double thresScore, int matchMethod, cv::Point &centerPoint, double &outAngle, double &matchScore);


#pragma endregion




#pragma region 内部函数区


#pragma endregion


};
