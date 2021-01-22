
#include "image_process.h"
#include <math.h>

ImageProcess::ImageProcess(QObject *parent) : QObject(parent)
{

}

ImageProcess::~ImageProcess()
{

}

#pragma region �ⲿ�ӿں�����


#pragma region  ��ȡָ��������㷨����
/*
*** ���polygon������
*** ����vpsΪ����Polygon�ı�Ե�㼯������Ĭ�ϵ㼯���ɵ�����Ϊ�������
*** regionΪ���������
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
*** ��þ��ε�����
*** ����point1Ϊ���ε����Ͻ�����㣬point2Ϊ�������½������
*** regionΪ���������
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
*** ���Բ�ε�����
*** ����point1ΪԲ�����ĵ����꣬radiusΪԲ�İ뾶
*** regionΪ���������
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
*** ���region�����ͼ��
*** ����srcImageΪԭͼ��regionΪĿ������
*** reduceDaomainΪ������������������
*/
bool ImageProcess::get_reduce_domain(cv::Mat srcImage, cv::Mat region, cv::Mat &reduceDaomain)
{
	srcImage.copyTo(reduceDaomain, region);
	return true;
}

/*
*** ������ı�Եֱ�߰����������ҽ��з���
*** ����linesΪ���б�Եֱ��
*** upLineΪ�����ˮƽֱ�ߣ�downLineΪ�����ˮƽֱ�ߣ�leftLineΪ������ֱֱ�ߣ�rightLineΪ�Ҳ���ֱֱ��
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
*** ���㷨fitLine�������Vec4f���͵�����ת��Ϊֱ���ϵ�point��
*** ����linesΪ���б�Եֱ��
*** point1Ϊֱ�ߵ�һ���㣬point2Ϊֱ�ߵڶ�����
*/
bool ImageProcess::convert_lineVec4f_to_point(cv::Vec4f vec4f, cv::Point &point1, cv::Point &point2)
{
	try
	{
		//��ȡ��бʽ�ĵ��б��
		cv::Point point0;
		point0.x = vec4f[2];
		point0.y = vec4f[3];

		double k = vec4f[1] / vec4f[0];

		if (abs(k) <= 1)
		{
			//����ֱ�ߵĶ˵�(y = k(x - x0) + y0)
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
*** ������ֱ�ߵĽ���
*** ����point1��point2�ֱ�Ϊ��һ��ֱ�ߵ���ʼ�����ֹ�㣬point3��point4�ֱ�Ϊ�ڶ���ֱ�ߵ���ʼ�����ֹ��
*** outCrossPointΪ����ֱ�ߵĽ����
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
*** ��ͼ����ж�ֵ��
*** ����srcImageΪ����ͼ��lowValueΪ��ֵ������ֵ��highValueΪ��ֵ������ֵ
*** dstThresholdImageΪĿ�������ֵ��ͼ��
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
*** ��ȡͼ��Ҷ�ֱ��ͼ����
*** ����srcImageΪ����ͼ��maskΪ������Ĥ
*** histoGraphΪ����Ҷ�ֱ��ͼͼ��histoDataΪ����Ҷ�ֱ��ͼ����
*/
bool ImageProcess::leo_getGrayImageHistData(cv::Mat srcImage, cv::Mat mask, cv::Mat &histoGraph, cv::Mat &histoData)
{
	try
	{
		//������ֱ��ͼ��ͨ����Ŀ����0��ʼ����
		int channels[] = { 0 };
		//����ֱ��ͼ����ÿһά�ϵĴ�С������Ҷ�ͼֱ��ͼ�ĺ�������ͼ��ĻҶ�ֵ����һά��bin�ĸ���
		//���ֱ��ͼͼ�������bin����Ϊx��������bin����Ϊy����channels[]={1,2}��ֱ��ͼӦ��Ϊ��ά�ģ�Z����ÿ��bin��ͳ�Ƶ���Ŀ
		const int histSize[] = { 256 };
		//ÿһάbin�ı仯��Χ
		float range[] = { 0,256 };

		//����bin�ı仯��Χ��������channelsӦ�ø�channelsһ��
		const float* ranges[] = { range };


		//opencv�м���ֱ��ͼ�ĺ�����hist��СΪ256*1��ÿ�д洢��ͳ�Ƶĸ��ж�Ӧ�ĻҶ�ֵ�ĸ���
		calcHist(&srcImage, 1, channels, mask, histoData, 1, histSize, ranges, true, false);//cv����cvCalcHist

																					   //�ҳ�ֱ��ͼͳ�Ƶĸ��������ֵ��������Ϊֱ��ͼ������ĸ�
		double maxValue = 0;
		//�Ҿ����������Сֵ����Ӧ�����ĺ���
		minMaxLoc(histoData, 0, &maxValue, 0, 0);
		//���ֵȡ��
		int rows = cvRound(maxValue);
		//����ֱ��ͼͼ��ֱ��ͼ������ĸ���Ϊ����������Ϊ256(�Ҷ�ֵ�ĸ���)
		//��Ϊ��ֱ��ͼ��ͼ�������Ժڰ���ɫΪ���֣���ɫΪֱ��ͼ��ͼ��

		cv::Mat histImage = cv::Mat::zeros(rows, 256, CV_8UC1);

		//ֱ��ͼͼ���ʾ
		for (int i = 0; i < 256; i++)
		{
			//ȡÿ��bin����Ŀ
			int temp = (int)(histoData.at<float>(i, 0));
			//���bin��ĿΪ0����˵��ͼ����û�иûҶ�ֵ��������Ϊ��ɫ
			//���ͼ�����иûҶ�ֵ���򽫸��ж�Ӧ������������Ϊ��ɫ
			if (temp)
			{
				//����ͼ�������������Ͻ�Ϊԭ�㣬����Ҫ���б任��ʹֱ��ͼͼ�������½�Ϊ����ԭ��
				histImage.col(i).rowRange(cv::Range(rows - temp, rows)) = 255;
			}
		}
		//����ֱ��ͼͼ���и߿��ܸܺߣ���˽���ͼ�����Ҫ���ж�Ӧ��������ʹֱ��ͼͼ���ֱ��
		resize(histImage, histoGraph, cv::Size(256, 256));
		return true;
	}
	catch (...)
	{
		return false;
	}


}

/*
*** ��������İٷֱȻ�ȡ��Ӧ�ٷֱȵĻҶ�ֵ
*** ����histoGraphDataΪ����ĻҶ�ֱ��ͼ���ݣ�percentValueΪ�ҶȰٷֱ�
*** grayValueΪֱ��ͼ��Ӧ�ٷֱ�λ�õĻҶ�ֵ
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
*** ������֮��ľ���
*** ����point1Ϊ����ĵ�һ���㣬point2Ϊ����ĵڶ�����
*** distanceΪ���
*/
bool ImageProcess::leo_getDistancs_p2p(cv::Point point1, cv::Point point2, double &distance)
{
	distance = powf((point1.x - point2.x), 2) + powf((point1.y - point2.y), 2);
	distance = sqrtf(distance);
	return true;
}

/*
*** ��㵽ֱ��֮��ľ���
*** ����PointDisΪֱ����һ�㣬pointLstart��pointLendΪֱ�ߵ������˵�
*** distanceΪ���
*/
bool ImageProcess::leo_getDistance_p2l(cv::Point PointDis, cv::Point pointLstart, cv::Point pointLend, double &distance)
{
	//��ֱ�߷���
	int A = 0, B = 0, C = 0;
	A = pointLstart.y - pointLend.y;
	B = pointLend.x - pointLstart.x;
	C = pointLstart.x*pointLend.y - pointLstart.y*pointLend.x;
	//����㵽ֱ�߾��빫ʽ
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
*** �����������Ŀ׶�
*** ����byteRegionΪ�����ֵͼ��
*** dstRegionΪ����Ľ��ͼ��
*/
bool ImageProcess::leo_fillRegion(cv::Mat byteRegion, cv::Mat &dstRegion)
{
	cv::Size m_Size = byteRegion.size();
	cv::Mat temimage = cv::Mat::zeros(m_Size.height + 2, m_Size.width + 2, byteRegion.type());      //��չͼ��,Ϊ��ˮ����ṩ�ռ�
	byteRegion.copyTo(temimage(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)));
	cv::floodFill(temimage, cv::Point(0, 0), cv::Scalar(255));
	cv::Mat cutImg;//�ü���չ��ͼ��  
	temimage(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)).copyTo(cutImg);
	dstRegion = byteRegion | (~cutImg);
	return true;
}

/*
*** ��״ѡ��ѡ���������������һ��
*** ����srcImageΪ����Դͼ�����ڴ�������ͼ��������С, allRegionContourΪ�������������Contour
*** maxRegionΪ������������
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
*** ͼ����ת
*** ����srcImageΪ����Դͼ��angleΪͼ����ת�Ƕȣ���ʱ��Ϊ��
*** outImageΪ�������ת���ͼ��
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
*** �������²���
*** ����imageΪ����Դͼ��, numLevelsΪ�������㼶
*** outImageΪ�������²������ͼ��
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
*** �ɱ�Ƕ��µ�ģ��ƥ��
*** ����srcImageΪ����Դͼ��, modelImageΪģ��ͼ��angleStartΪ��ʼ�Ƕȣ�angleRangeΪ�Ƕȷ�Χ��
	angleStepΪ�ǶȲ�����numLevelsΪ������������thresScore�������С�÷�����matchMethodģ��ƥ�䷽����

*** centerPointģ���������꣬outAngleΪģ����ת�Ƕȣ� matchScoreΪģ��ƥ��÷�
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

	//����ͼ��ƥ������Ҫ�Ĳ���
	int resultData_cols = srcImage.cols - modelImage.cols + 1;
	int resultData_rows = srcImage.rows - modelImage.rows + 1;
	cv::Mat resultData = cv::Mat(resultData_rows, resultData_cols, CV_32FC1);
	cv::Mat src, model;
	srcImage.copyTo(src);
	modelImage.copyTo(model);

	//��ģ��ͼ��ʹ����ͼ��ֱ����ͼ��������²���
	for (size_t i = 0; i < numLevels; i++)
	{
		cv::pyrDown(src, src, cv::Size(src.cols / 2, src.rows / 2));
		cv::pyrDown(model, model, cv::Size(model.cols / 2, model.rows / 2));
	}

	//��û����ת������½��е�һ��ƥ��
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

	//�����ƥ�������ʮ���ǶȲ���1����ѭ��ƥ�䣬ֱ���ǶȲ���С�ڲ����ǶȲ���
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





#pragma region �ڲ�������



#pragma endregion