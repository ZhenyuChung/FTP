#include "QualityMap.h"

/*******************************************************/
/*汉宁窗滤波形成quality map
**输入：复数矩阵
**操作：汉宁窗滤波，取幅度值作为quality map
**输出：quality map
*/
void hamingFilter(Mat *contourFT, Mat *qualityMap)
{
	//建立haning window
	Mat filter = Mat::zeros((*contourFT).size(), CV_32F);
	//全部条纹
	//Mat hanningWindow = filter(Rect(filter.rows / 2 +50, filter.cols / 2 - 15, 25, 30));
	//createHanningWindow(hanningWindow, Size(25, 30), CV_32F);
	//脸部条纹
	Mat hanningWindow = filter(Rect(filter.rows / 2 + 25, filter.cols / 2 - 10, 15, 20));
	createHanningWindow(hanningWindow, Size(15, 20), CV_32F);

	int cx = filter.cols / 2;
	int cy = filter.rows / 2;
	Mat q0(filter, Rect(0, 0, cx, cy));
	Mat q1(filter, Rect(cx, 0, cx, cy));
	Mat q2(filter, Rect(0, cy, cx, cy));
	Mat q3(filter, Rect(cx, cy, cx, cy));
	Mat tmp;
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);
	q1.copyTo(tmp);
	q2.copyTo(q1);
	tmp.copyTo(q2);
	//write_txt("filter.txt", filter);

	//将傅里叶变换后复合矩阵分为两个单通道矩阵
	Mat planes[] = { Mat::zeros((*contourFT).size(), CV_32F) , Mat::zeros((*contourFT).size(), CV_32F) };
	split((*contourFT), planes);

	//滤波器与傅里叶变换后矩阵相乘，只保留基频分量
	planes[0] = planes[0].mul(filter);
	planes[1] = planes[1].mul(filter);
	//取共轭，为逆变换做准备
	planes[1] = planes[1].mul(-1);

	//重新合成复合矩阵
	merge(planes, 2, (*contourFT));

	Mat contourIFT;
	IFT(*contourFT, &contourIFT);
	split(contourIFT, planes);
	magnitude(planes[0], planes[1], *qualityMap);
	//write_txt("qualityMapa.txt", *quality_Map);

	normalize(*qualityMap, *qualityMap, 0, 1, NORM_MINMAX);
	imshow("quality map", *qualityMap);
	cout << "quality map finished" << endl;
	return;
}


/*******************************************************/
/*计算phase derivative variance求quality map
**输入：包裹相位矩阵
**操作：计算phase derivative variance,即求该点与八邻点的差值的均方差
**输出：quality map
*/
void phaseDerivativeVariance(Mat *phaseData, Mat *qualityMap)
{
	*qualityMap = Mat::zeros(phaseData->size(), CV_32F);

	for (int i = 0; i < qualityMap->rows; i++)
	{
		for (int j = 0; j < qualityMap->cols; j++)
		{
			if ((i != 0) && (j != 0) && (i != (qualityMap->rows - 1)) && (j != (qualityMap->cols - 1)))
			{
				int n = 0;
				float grad[4] = { 0.0 };

				for (int x = -1; x <= 1; x++)
				{
					for (int y = -1; y <= 1; y++)
					{
						if (abs(x) != abs(y))
						{
							grad[n] = gradient(phaseData->ptr<float>(i)[j], phaseData->ptr<float>(i + x)[j + y]);
							n++; 
						}
					
					}
				}
				qualityMap->ptr<float>(i)[j] = standardDiff(grad);
			}
		}
	}
	return;
}


/*******************************************************/
/*计算两个点的相位差
**输入：两个相位值
**操作：计算两个点的相位差
**输出：相位差
*/
float gradient(float p1, float p2)
{
	float dst = p1 - p2;
	if (dst > 0.5)
		dst -= 1.0;
	else if (dst < -0.5)
		dst += 1.0;

	return dst;
}


/*******************************************************/
/*计算数组的均方差
**输入：数组
**操作：计算数组的均方差
**输出：均方差
*/
float standardDiff(float *grad)
{
	float avg = 0.0;
	float sum = 0.0;
	for (int a = 0; a < 4; a++)
	{
		avg += grad[a];
	}
	avg = avg / 4;

	for (int a = 0; a < 4; a++)
	{
		sum = (grad[a] - avg)*(grad[a] - avg);
	}

	return sqrtf(sum / 4);
}