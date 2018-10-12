#pragma once
#include <opencv2\opencv.hpp>
#include <iostream>
#include <math.h>
#include <time.h>
#include <fstream>
#include "QGPU.h"
#include "QualityMap.h"
#include "Residue.h"

using namespace cv;
using namespace std;

const float pi = 3.1415926;

//��ȡ���������ξ���
void readCalibrationFile(Mat &intrinsic, Mat &distortion);

//��ͼ������ɢ����Ҷ�任
void FT(Mat contour0, Mat *contour);

//�Ը���Ҷ�任���ͼ���˲����˳���Ƶ����
void filter(Mat *contour);

//��ͼ������ɢ����Ҷ�任
void IFT(Mat contourFT, Mat *contourIFT);

//����λ��
Mat dstPhase(Mat ref, Mat obj, Mat *qualityMap);

//�������λ
void unwrappPhase(Mat *phase);

//��߶�
Mat getHeight(float L, float d, Mat phase);

//д��txt�ļ�
void write_txt(char *name, Mat data);