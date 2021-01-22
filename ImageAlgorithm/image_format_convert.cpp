#include "image_format_convert.h"

ImageFormatConvert::ImageFormatConvert(QObject *parent) :QObject(parent)
{

}

ImageFormatConvert::~ImageFormatConvert()
{

}

void ImageFormatConvert::mat2qimage(cv::Mat srcImage, QImage &dstImage)
{
	if (srcImage.empty())
	{
		return;
	}
	QImage m_QImg;
	switch (srcImage.channels())
	{
	case 3:
		cvtColor(srcImage, srcImage, CV_BGR2RGB);
		m_QImg = QImage(static_cast<uchar *>(srcImage.data), srcImage.cols, srcImage.rows, QImage::Format_RGB888);

		break;
	case 4:
		m_QImg = QImage(static_cast<uchar *>(srcImage.data), srcImage.cols, srcImage.rows, QImage::Format_ARGB32);
		break;
	default:
		m_QImg = QImage(srcImage.cols, srcImage.rows, QImage::Format_Indexed8);
		uchar *matData = srcImage.data;
		for (int i = 0; i < srcImage.rows; i++)
		{
			uchar *rowData = m_QImg.scanLine(i);
			memcpy(rowData, matData, srcImage.cols);
			matData += srcImage.cols;
		}
	}
	dstImage = m_QImg.copy(0, 0, srcImage.cols, srcImage.rows);
}

void ImageFormatConvert::qimage2mat(QImage srcImage, cv::Mat &dstImage)
{
	cv::Mat mat;
	switch (srcImage.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		mat = cv::Mat(srcImage.height(), srcImage.width(), CV_8UC4, (void*)srcImage.bits(), srcImage.bytesPerLine());
		break;
	case QImage::Format_RGB888:
		mat = cv::Mat(srcImage.height(), srcImage.width(), CV_8UC3, (void*)srcImage.bits(), srcImage.bytesPerLine());
		cv::cvtColor(mat, mat, CV_BGR2RGB);
		break;
	case QImage::Format_Indexed8:
		mat = cv::Mat(srcImage.height(), srcImage.width(), CV_8UC1, (void*)srcImage.bits(), srcImage.bytesPerLine());
		break;
	}
	
	mat.copyTo(dstImage);
}