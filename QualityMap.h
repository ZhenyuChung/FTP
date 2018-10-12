#pragma once
#include<opencv2\opencv.hpp>
#include <iostream>
#include <math.h>
#include "FTP.h"

using namespace cv;
using namespace std;

//�������˲��γ�quality map
void hamingFilter(Mat *contourFT, Mat *qualityMap);

//����phase derivative variance��quality map
void phaseDerivativeVariance(Mat *phaseData, Mat *qualityMap);

//�������������λ��
float gradient(float p1, float p2);

//��������ľ�����
float standardDiff(float *grad);