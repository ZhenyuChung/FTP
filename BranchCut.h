#pragma once
#include<opencv2\opencv.hpp>
#include <iostream>
#include <math.h>
#include <vector>
#include "Residue.h"

using namespace cv;
using namespace std;

//Branch cut连接residues
void BranchCut(Mat residueMap, vector<residueFlag> residueList, Mat *Branch);

//寻找规定范围内的residue
bool searchResidue(Point residuePoint, int Boxize, vector<Point> *neighbourResidue);

//在两个residues之间放入branch
void placeBranch(Point p1, Point p2, Mat *Branch);
