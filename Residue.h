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

//residue��
class residueFlag
{
private:
	Point residuePosition;
	int charge;
	bool activeFlag;

public:
	//���캯������ʼ��
	residueFlag(Point residuePosition, int charge) { this->residuePosition = residuePosition; this->charge = charge; activeFlag = Active; }

	//activie״̬��Ϊinactive
	void inactive() { activeFlag = Inactive; }

	//���ص�����
	Point residuePoint() { return residuePosition; }

	//����chargeֵ
	int chargeVal() { return charge; }

	//���ص�ǰactive״̬
	int activityOfResidue() { return activeFlag; }
};


//Ѱ��residue��
vector<residueFlag> residue(Mat phase, Mat *residueMap);