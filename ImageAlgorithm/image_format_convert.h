#pragma once

#include "opencv2\opencv.hpp"
#include <QObject>
#include <QImage>

class ImageFormatConvert : QObject
{
	Q_OBJECT
public:
	ImageFormatConvert(QObject *parent=Q_NULLPTR);
	~ImageFormatConvert();

public:
	virtual void mat2qimage(cv::Mat srcImage, QImage &dstImage);
	virtual void qimage2mat(QImage srcImage, cv::Mat &dstImage);


};
