#pragma once
#include<opencv2\opencv.hpp>
#include <iostream>
#include <math.h>
#include <vector>
#include "Residue.h"

using namespace cv;
using namespace std;

//Branch cut����residues
void BranchCut(Mat residueMap, vector<residueFlag> residueList, Mat *Branch);

//Ѱ�ҹ涨��Χ�ڵ�residue
bool searchResidue(Point residuePoint, int Boxize, vector<Point> *neighbourResidue);

//������residues֮�����branch
void placeBranch(Point p1, Point p2, Mat *Branch);
