#pragma once
#include<opencv2\opencv.hpp>
#include <iostream>
#include <math.h>
#include <vector>

using namespace cv;
using namespace std;

//residue charge
#define positive 1
#define negative 0
#define normal -1

//activity of residue
#define Active 1
#define Inactive 0

//residue类
class residueFlag
{
private:
	Point residuePosition;
	int charge;
	bool activeFlag;

public:
	//构造函数，初始化
	residueFlag(Point residuePosition, int charge) { this->residuePosition = residuePosition; this->charge = charge; activeFlag = Active; }

	//activie状态改为inactive
	void inactive() { activeFlag = Inactive; }

	//返回点坐标
	Point residuePoint() { return residuePosition; }

	//返回charge值
	int chargeVal() { return charge; }

	//返回当前active状态
	int activityOfResidue() { return activeFlag; }
};


//寻找residue点
vector<residueFlag> residue(Mat phase, Mat *residueMap);