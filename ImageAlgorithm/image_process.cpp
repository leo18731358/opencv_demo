
#include "image_process.h"
#include <math.h>

ImageProcess::ImageProcess(QObject *parent) : QObject(parent)
{

}

ImageProcess::~ImageProcess()
{

}

#pragma region 外部接口函数区


#pragma region  获取指定区域的算法函数
/*
*** 获得polygon的区域
*** 其中vps为单个Polygon的边缘点集，函数默认点集构成的轮廓为封闭轮廓
*** region为输出的区域
*/
bool ImageProcess::get_region(cv::Mat srcImage, std::vector<cv::Point> vps, cv::Mat &region)
{
	if (srcImage.empty() || vps.empty())
	{
		return false;
	}
	if (vps.size() < 3)
	{
		return false;
	}
	try
	{
		std::vector<std::vector<cv::Point>> maskArea;
		maskArea.push_back(vps);
		srcImage.copyTo(region);
		region.setTo(cv::Scalar::all(0));
		cv::fillPoly(region, maskArea, cv::Scalar::all(255));
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/*
*** 获得矩形的区域
*** 其中point1为矩形的左上角坐标点，point2为矩形右下角坐标点
*** region为输出的区域
*/
bool ImageProcess::get_region(cv::Mat srcImage, cv::Point point1, cv::Point point2, cv::Mat &region)
{
	if (srcImage.empty())
	{
		return false;
	}
	try
	{
		srcImage.copyTo(region);
		region.setTo(cv::Scalar::all(0));
		cv::rectangle(region, point1, point2, cv::Scalar::all(255), -1);
		return true;
	}
	catch (...)
	{
		return false;
	}
}

/*
*** 获得圆形的区域
*** 其中point1为圆的中心点坐标，radius为圆的半径
*** region为输出的区域
*/
bool ImageProcess::get_region(cv::Mat srcImage, cv::Point point1, float radius, cv::Mat &region)
{
	if (srcImage.empty())
	{
		return false;
	}
	try
	{
		srcImage.copyTo(region);
		region.setTo(cv::Scalar::all(0));
		cv::circle(region, point1, radius, cv::Scalar::all(255), -1);
		return true;
	}
	catch (...)
	{
		return false;
	}

}

#pragma endregion

/*
*** 获得region区域的图像
*** 其中srcImage为原图，region为目标区域
*** reduceDaomain为输出的区域输出的区域
*/
bool ImageProcess::get_reduce_domain(cv::Mat srcImage, cv::Mat region, cv::Mat &reduceDaomain)
{
	srcImage.copyTo(reduceDaomain, region);
	return true;
}

/*
*** 将区域的边缘直线按照上下左右进行分类
*** 其中lines为所有边缘直线
*** upLine为上面的水平直线，downLine为下面的水平直线，leftLine为左侧的竖直直线，rightLine为右侧竖直直线
*/
bool ImageProcess::get_horizontal_and_vertical_line(std::vector<cv::Vec4f> lines, std::vector<cv::Point> &upLine,
	std::vector<cv::Point> &downLine, std::vector<cv::Point> &leftLine, std::vector<cv::Point> &rightLine)
{
	std::vector<cv::Vec4f> upLineV, downLineV, leftLineV, rightLineV, hLineV, vLineV;
	std::vector<double> x_value, y_value;
	for (size_t i = 0; i < lines.size(); i++)
	{
		if (lines[i][0] - lines[i][2] == 0)
		{
			vLineV.push_back(lines[i]);
			x_value.push_back((lines[i][0] + lines[i][2]) / 2);
			continue;
		}
		else if (lines[i][1] - lines[i][3] == 0)
		{
			hLineV.push_back(lines[i]);
			y_value.push_back((lines[i][1] + lines[i][3]) / 2);
			continue;
		}

		double k_value = (lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
		double angle_value = atan(k_value) / CV_PI * 180;
		if (abs(angle_value) >= 45)
		{
			vLineV.push_back(lines[i]);
			x_value.push_back((lines[i][0] + lines[i][2]) / 2);
		}
		else
		{
			hLineV.push_back(lines[i]);
			y_value.push_back((lines[i][1] + lines[i][3]) / 2);
		}
	}

	if (!x_value.empty())
	{
		std::sort(x_value.begin(), x_value.end());
		double average_x_value = (x_value[0] + x_value[x_value.size() - 1]) / 2;
		for (size_t i = 0; i < vLineV.size(); i++)
		{
			if ((vLineV[i][0] + vLineV[i][2]) / 2 > average_x_value)
			{
				rightLineV.push_back(vLineV[i]);
			}
			else
			{
				leftLineV.push_back(vLineV[i]);
			}
		}
	}
	
	if (!y_value.empty())
	{
		std::sort(y_value.begin(), y_value.end());
		double average_y_value = (y_value[0] + y_value[y_value.size() - 1]) / 2;
		for (size_t i = 0; i < hLineV.size(); i++)
		{
			if ((hLineV[i][1] + hLineV[i][3]) / 2 >= average_y_value)
			{
				downLineV.push_back(hLineV[i]);
			}
			else
			{
				upLineV.push_back(hLineV[i]);
			}
		}
	}

	for (size_t i = 0; i < upLineV.size(); i++)
	{
		upLine.push_back(cv::Point(upLineV[i][0], upLineV[i][1]));
		upLine.push_back(cv::Point(upLineV[i][2], upLineV[i][3]));
	}

	for (size_t i = 0; i < downLineV.size(); i++)
	{
		downLine.push_back(cv::Point(downLineV[i][0], downLineV[i][1]));
		downLine.push_back(cv::Point(downLineV[i][2], downLineV[i][3]));
	}

	for (size_t i = 0; i < leftLineV.size(); i++)
	{
		leftLine.push_back(cv::Point(leftLineV[i][0], leftLineV[i][1]));
		leftLine.push_back(cv::Point(leftLineV[i][2], leftLineV[i][3]));
	}

	for (size_t i = 0; i < rightLineV.size(); i++)
	{
		rightLine.push_back(cv::Point(rightLineV[i][0], rightLineV[i][1]));
		rightLine.push_back(cv::Point(rightLineV[i][2], rightLineV[i][3]));
	}
	return true;
}

/*
*** 将算法fitLine求出来的Vec4f类型的数据转换为直线上的point点
*** 其中lines为所有边缘直线
*** point1为直线第一个点，point2为直线第二个点
*/
bool ImageProcess::convert_lineVec4f_to_point(cv::Vec4f vec4f, cv::Point &point1, cv::Point &point2)
{
	try
	{
		//获取点斜式的点和斜率
		cv::Point point0;
		point0.x = vec4f[2];
		point0.y = vec4f[3];

		double k = vec4f[1] / vec4f[0];

		if (abs(k) <= 1)
		{
			//计算直线的端点(y = k(x - x0) + y0)
			point1.x = 0;
			point1.y = k * (0 - point0.x) + point0.y;
			point2.x = point0.x + 1000;
			point2.y = k * (point0.x + 1000 - point0.x) + point0.y;
		}
		else
		{
			point1.x = -point0.y / k + point0.x;
			point1.y = 0;
			point2.x = (point0.y + 1000 - point0.y)/k + point0.x;
			point2.y = point0.y + 1000;
			
		}

		return true;
	}
	catch (...)
	{
		return false;
	}
}

/*
*** 求两条直线的交点
*** 其中point1、point2分别为第一条直线的起始点和终止点，point3、point4分别为第二条直线的起始点和终止点
*** outCrossPoint为两条直线的交叉点
*/
bool ImageProcess::get_cross_point(cv::Point point1, cv::Point point2, cv::Point point3, cv::Point point4,
	cv::Point &outCrossPoint)
{
	double x1 = point2.x - point1.x;
	double y1 = point2.y - point1.y;
	double x2 = point4.x - point3.x;
	double y2 = point4.y - point3.y;

	double x21 = point3.x - point1.x;
	double y21 = point3.y - point1.y;

	double distance = y1*x2 - y2*x1;
	if (0 == distance)
	{
		outCrossPoint = cv::Point(-1, -1);
		return false;
	}
	outCrossPoint.x = (x1*x2*y21 + y1*x2*point1.x - y2*x1*point3.x) / distance;
	outCrossPoint.y = -(y1*y2*x21 + x1*y2*point1.y - x2*y1*point3.y) / distance;

	return true;

}


/*
*** 对图像进行二值化
*** 其中srcImage为输入图像，lowValue为二值化低阈值，highValue为二值化高阈值
*** dstThresholdImage为目标区域二值化图像
*/
bool ImageProcess::leo_threshold(cv::Mat srcImage, cv::Mat &dstThresholdImage, int lowValue, int highValue)
{
	if (!srcImage.data)
	{
		return false;
	}
	if (lowValue >= highValue || lowValue < 0 || highValue > 255)
	{
		lowValue = 0;
		highValue = 255;
	}
	cv::Mat dstImage1, dstImage2;
	cv::threshold(srcImage, dstImage1, lowValue, 255, cv::THRESH_BINARY_INV);
	cv::threshold(srcImage, dstImage2, highValue, 255, cv::THRESH_BINARY_INV);
	dstThresholdImage = dstImage2 - dstImage1;
	return true;
}

/*
*** 获取图像灰度直方图数据
*** 其中srcImage为输入图像，mask为输入掩膜
*** histoGraph为输出灰度直方图图像，histoData为输出灰度直方图数据
*/
bool ImageProcess::leo_getGrayImageHistData(cv::Mat srcImage, cv::Mat mask, cv::Mat &histoGraph, cv::Mat &histoData)
{
	try
	{
		//定义求直方图的通道数目，从0开始索引
		int channels[] = { 0 };
		//定义直方图的在每一维上的大小，例如灰度图直方图的横坐标是图像的灰度值，就一维，bin的个数
		//如果直方图图像横坐标bin个数为x，纵坐标bin个数为y，则channels[]={1,2}其直方图应该为三维的，Z轴是每个bin上统计的数目
		const int histSize[] = { 256 };
		//每一维bin的变化范围
		float range[] = { 0,256 };

		//所有bin的变化范围，个数跟channels应该跟channels一致
		const float* ranges[] = { range };


		//opencv中计算直方图的函数，hist大小为256*1，每行存储的统计的该行对应的灰度值的个数
		calcHist(&srcImage, 1, channels, mask, histoData, 1, histSize, ranges, true, false);//cv中是cvCalcHist

																					   //找出直方图统计的个数的最大值，用来作为直方图纵坐标的高
		double maxValue = 0;
		//找矩阵中最大最小值及对应索引的函数
		minMaxLoc(histoData, 0, &maxValue, 0, 0);
		//最大值取整
		int rows = cvRound(maxValue);
		//定义直方图图像，直方图纵坐标的高作为行数，列数为256(灰度值的个数)
		//因为是直方图的图像，所以以黑白两色为区分，白色为直方图的图像

		cv::Mat histImage = cv::Mat::zeros(rows, 256, CV_8UC1);

		//直方图图像表示
		for (int i = 0; i < 256; i++)
		{
			//取每个bin的数目
			int temp = (int)(histoData.at<float>(i, 0));
			//如果bin数目为0，则说明图像上没有该灰度值，则整列为黑色
			//如果图像上有该灰度值，则将该列对应个数的像素设为白色
			if (temp)
			{
				//由于图像坐标是以左上角为原点，所以要进行变换，使直方图图像以左下角为坐标原点
				histImage.col(i).rowRange(cv::Range(rows - temp, rows)) = 255;
			}
		}
		//由于直方图图像列高可能很高，因此进行图像对列要进行对应的缩减，使直方图图像更直观
		resize(histImage, histoGraph, cv::Size(256, 256));
		return true;
	}
	catch (...)
	{
		return false;
	}


}

/*
*** 根据输入的百分比获取对应百分比的灰度值
*** 其中histoGraphData为输入的灰度直方图数据，percentValue为灰度百分比
*** grayValue为直方图对应百分比位置的灰度值
*/
bool ImageProcess::leo_getPercentHistoValue(cv::Mat histoGraphData, float percentValue, int &grayValue)
{
	if (percentValue > 1 || percentValue < 0)
	{
		percentValue = 0;
	}
	if (histoGraphData.data)
	{
		double sumValue = 0.0;
		for (size_t i = 0; i < histoGraphData.rows; i++)
		{
			sumValue += histoGraphData.at<float>(i, 0);
		}
		double percentSum = 0.0;
		for (size_t i = 0; i < histoGraphData.rows; i++)
		{
			percentSum += histoGraphData.at<float>(i, 0);
			if (percentSum/ sumValue > percentValue)
			{
				grayValue = i;
				break;
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}

/*
*** 求两点之间的距离
*** 其中point1为输入的第一个点，point2为输入的第二个点
*** distance为结果
*/
bool ImageProcess::leo_getDistancs_p2p(cv::Point point1, cv::Point point2, double &distance)
{
	distance = powf((point1.x - point2.x), 2) + powf((point1.y - point2.y), 2);
	distance = sqrtf(distance);
	return true;
}

/*
*** 求点到直线之间的距离
*** 其中PointDis为直线外一点，pointLstart、pointLend为直线的两个端点
*** distance为结果
*/
bool ImageProcess::leo_getDistance_p2l(cv::Point PointDis, cv::Point pointLstart, cv::Point pointLend, double &distance)
{
	//求直线方程
	int A = 0, B = 0, C = 0;
	A = pointLstart.y - pointLend.y;
	B = pointLend.x - pointLstart.x;
	C = pointLstart.x*pointLend.y - pointLstart.y*pointLend.x;
	//代入点到直线距离公式
	if (A==0 && B==0)
	{
		distance = 0;
		return false;
	}
	else
	{
		distance = ((float)abs(A*PointDis.x + B*PointDis.y + C)) / ((float)sqrtf(A*A + B*B));
		return true;
	}	
}

/*
*** 填充区域里面的孔洞
*** 其中byteRegion为区域二值图像
*** dstRegion为填充后的结果图像
*/
bool ImageProcess::leo_fillRegion(cv::Mat byteRegion, cv::Mat &dstRegion)
{
	cv::Size m_Size = byteRegion.size();
	cv::Mat temimage = cv::Mat::zeros(m_Size.height + 2, m_Size.width + 2, byteRegion.type());      //延展图像,为漫水填充提供空间
	byteRegion.copyTo(temimage(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)));
	cv::floodFill(temimage, cv::Point(0, 0), cv::Scalar(255));
	cv::Mat cutImg;//裁剪延展的图像  
	temimage(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)).copyTo(cutImg);
	dstRegion = byteRegion | (~cutImg);
	return true;
}

/*
*** 形状选择，选择区域中面积最大的一个
*** 其中srcImage为输入源图像，用于创建区域图像的总体大小, allRegionContour为输入的所有区域Contour
*** maxRegion为输出的最大区域
*/
bool ImageProcess::leo_selectShapeMax(cv::Mat srcImage, std::vector<std::vector<cv::Point>> allRegionContour, cv::Mat &maxRegion)
{
	if (0 == allRegionContour.size())
	{
		return false;
	}
	std::vector<double> allContourArea;
	for (size_t i = 0; i < allRegionContour.size(); i++)
	{
		allContourArea.push_back(cv::contourArea(allRegionContour[i]));
	}
	std::vector<double>::iterator pos = std::max_element(allContourArea.begin(), allContourArea.end());
	int objIndex = std::distance(allContourArea.begin(), pos);
	srcImage.copyTo(maxRegion);
	maxRegion.setTo(0);
	std::vector<std::vector<cv::Point>> maxRegionPoint;
	maxRegionPoint.push_back(allRegionContour[objIndex]);
	cv::fillPoly(maxRegion, maxRegionPoint, cv::Scalar::all(255));
	return true;

}

/*
*** 图像旋转
*** 其中srcImage为输入源图像，angle为图像旋转角度，逆时针为正
*** outImage为输出的旋转后的图像
*/
bool ImageProcess::leo_imageRotate(cv::Mat image, double angle, cv::Mat &outImage)
{
	if (!image.data)
	{
		return false;
	}
	cv::Point2f rotateCenter = cv::Point2f(static_cast<float>(image.cols / 2), static_cast<float>(image.rows / 2));
	cv::Mat rotateMatrix = cv::getRotationMatrix2D(rotateCenter, angle, 1);
	cv::warpAffine(image, outImage, rotateMatrix, image.size());
	return true;
}

/*
*** 金字塔下采样
*** 其中image为输入源图像, numLevels为金字塔层级
*** outImage为金字塔下采样后的图像
*/
bool ImageProcess::leo_imagePyrDown(cv::Mat image, int numLevels,cv::Mat &outImage)
{
	if (!image.data)
	{
		return false;
	}
	for (size_t i = 0; i < numLevels; i++)
	{
		cv::pyrDown(image, outImage, cv::Size(image.cols / 2, image.rows / 2));
	}
	return true;
}

/*
*** 可变角度下的模板匹配
*** 其中srcImage为输入源图像, modelImage为模板图像，angleStart为起始角度，angleRange为角度范围，
	angleStep为角度步长，numLevels为金字塔层数，thresScore需求的最小得分数，matchMethod模板匹配方法，

*** centerPoint模板中心坐标，outAngle为模板旋转角度， matchScore为模板匹配得分
*/
bool ImageProcess::leo_multiAngleMatchTemplate(cv::Mat srcImage, cv::Mat modelImage, double angleStart, double angleRange, double angleStep, int numLevels, double thresScore, int matchMethod, cv::Point &centerPoint, double &outAngle, double &matchScore)
{
	if (!srcImage.data)
	{
		return false;
	}
	double step = angleStep / ((angleRange / angleStep) / 100);
	double start = angleStart;
	double range = angleRange;

	//定义图像匹配所需要的参数
	int resultData_cols = srcImage.cols - modelImage.cols + 1;
	int resultData_rows = srcImage.rows - modelImage.rows + 1;
	cv::Mat resultData = cv::Mat(resultData_rows, resultData_cols, CV_32FC1);
	cv::Mat src, model;
	srcImage.copyTo(src);
	modelImage.copyTo(model);

	//对模板图像和待检测图像分别进行图像金字塔下采样
	for (size_t i = 0; i < numLevels; i++)
	{
		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));
		cv::pyrDown(model, model, cv::Size(model.cols / 2, model.rows / 2));
	}

	//在没有旋转的情况下进行第一次匹配
	cv::matchTemplate(src, model, resultData, matchMethod);
	double minVal = -1;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	cv::minMaxLoc(resultData, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
	
	cv::Point location = maxLoc;
	double tempScore;
	if (0 == matchMethod || 1 == matchMethod)
	{
		tempScore = minVal;
	}
	else
	{
		tempScore = maxVal;
	}
	double tempAngle = 0;

	cv::Mat newImage;

	//以最佳匹配点左右十倍角度步长1进行循环匹配，直到角度步长小于参数角度步长
	if (0 == matchMethod || 1 == matchMethod)
	{
		do
		{
			for (size_t i = 0; i <= (int)range/step; i++)
			{
				leo_imageRotate(model, start + step*i, newImage);
				cv::matchTemplate(src, newImage, resultData, matchMethod);
				cv::minMaxLoc(resultData, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
				if (minVal< tempScore)
				{
					location = maxLoc;
					tempScore = minVal;
					tempAngle = start + start*i;
				}
			}
			range = step * 2;
			start = tempAngle - step;
			step = step / 10;
		} while (step > angleStep);
		if (tempScore < thresScore)
		{
			centerPoint.x = location.x * pow(2, numLevels) + modelImage.cols / 2;
			centerPoint.y = location.y * pow(2, numLevels) + modelImage.rows / 2;
			outAngle = -tempAngle;
			matchScore = tempScore;
			return true;
		}
		else
		{
			centerPoint.x = 0;
			centerPoint.y = 0;
			outAngle = 0;
			matchScore = -1;
			return false;
		}

	}
	else
	{
		do
		{
			for (size_t i = 0; i <= (int)range/step; i++)
			{
				leo_imageRotate(model, start + step*i, newImage);
				cv::matchTemplate(src, newImage, resultData, matchMethod);
				cv::minMaxLoc(resultData, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
				if (maxVal > tempScore)
				{
					location = maxLoc;
					tempScore = maxVal;
					tempAngle = start + start*i;
				}
			}
			range = step * 2;
			start = tempAngle - step;
			step = step / 10;
		} while (step > angleStep);
		if (tempScore > thresScore)
		{
			centerPoint.x = location.x * pow(2, numLevels) + modelImage.cols/2;
			centerPoint.y = location.y * pow(2, numLevels) + modelImage.rows/2;
			outAngle = -tempAngle;
			matchScore = tempScore;
			return true;
		}
		else
		{
			centerPoint.x = 0;
			centerPoint.y = 0;
			outAngle = 0;
			matchScore = -1;
			return false;
		}
	}

}

#pragma endregion





#pragma region 内部函数区



#pragma endregion