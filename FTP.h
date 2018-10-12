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

//读取摄像机内外参矩阵
void readCalibrationFile(Mat &intrinsic, Mat &distortion);

//对图像做离散傅里叶变换
void FT(Mat contour0, Mat *contour);

//对傅里叶变换后的图像滤波，滤出基频分量
void filter(Mat *contour);

//对图像做离散傅里叶变换
void IFT(Mat contourFT, Mat *contourIFT);

//求相位差
Mat dstPhase(Mat ref, Mat obj, Mat *qualityMap);

//解包裹相位
void unwrappPhase(Mat *phase);

//求高度
Mat getHeight(float L, float d, Mat phase);

//写入txt文件
void write_txt(char *name, Mat data);