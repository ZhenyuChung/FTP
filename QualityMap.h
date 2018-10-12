#pragma once
#include<opencv2\opencv.hpp>
#include <iostream>
#include <math.h>
#include "FTP.h"

using namespace cv;
using namespace std;

//汉宁窗滤波形成quality map
void hamingFilter(Mat *contourFT, Mat *qualityMap);

//计算phase derivative variance求quality map
void phaseDerivativeVariance(Mat *phaseData, Mat *qualityMap);

//计算两个点的相位差
float gradient(float p1, float p2);

//计算数组的均方差
float standardDiff(float *grad);