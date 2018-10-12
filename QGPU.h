#pragma once
#include <opencv2\opencv.hpp>
#include <iostream>
#include <queue>
#include <math.h>
#include "FTP.h"

using namespace cv;
using namespace std;

//质量图指导解包裹
void guidingUnwrap(Mat *phase, Mat *qualityMap);

//用于生成优先队列的节点
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

//用于判断对应位置是否进队，是否已解包裹
enum maskState
{
	Wrapped,//未解包裹,0
	Unwrapped//已解包裹,1
};

//将一点的四个邻点进队
void push(Mat *phase, priority_queue<Node_> *guiding, Point *point, Mat *qualityMap, Mat *mask, Mat *n);

//某点解包裹
void unwrap(Mat *phase, Mat *mask, Point seed, Point *point, Mat *n);


//InSAR解包裹
void InSARunwrap(Mat *phase, Mat *mask, Point *point);

//Mat类型求和
float sumMat(Mat m);