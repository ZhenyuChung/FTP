#pragma once
#include <opencv2\opencv.hpp>
#include <iostream>
#include <queue>
#include <math.h>
#include "FTP.h"

using namespace cv;
using namespace std;

//����ͼָ�������
void guidingUnwrap(Mat *phase, Mat *qualityMap);

//�����������ȶ��еĽڵ�
struct Node_
{
	Point point;
	float val;
	Node_(Point p, float v) :point(p), val(v) {};
	friend bool operator < (const struct Node_ &n1, const struct Node_ &n2)
	{
		return n1.val < n2.val;
	}
};

//�����ж϶�Ӧλ���Ƿ���ӣ��Ƿ��ѽ����
enum maskState
{
	Wrapped,//δ�����,0
	Unwrapped//�ѽ����,1
};

//��һ����ĸ��ڵ����
void push(Mat *phase, priority_queue<Node_> *guiding, Point *point, Mat *qualityMap, Mat *mask, Mat *n);

//ĳ������
void unwrap(Mat *phase, Mat *mask, Point seed, Point *point, Mat *n);


//InSAR�����
void InSARunwrap(Mat *phase, Mat *mask, Point *point);

//Mat�������
float sumMat(Mat m);