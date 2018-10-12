#include "Residue.h"
#include "QualityMap.h"

/*******************************************************/
/*寻找residue点
**输入：相位矩阵
**操作：
**输出：residue map
**reference:
**[1]Donald J. Bone.Fourier fringe analysis: the two-dimensional phase unwrapping problem[J].Applied Optics,1991,Vol.30(25): 3627-3632
**[2]D Ghiglia, M Pritt.Two Dimensional Phase Unwrapping: Theory, Algorithms and Software[J].
*/
vector<residueFlag> residue(Mat phase, Mat *residueMap)
{
	*residueMap = Mat::zeros(phase.size(),CV_32FC3);
	vector<residueFlag> residueList;
	
	for (int i = 0; i < (phase.rows - 1); i++)
		for (int j = 0, b = 0; j < (phase.cols - 1); j++, b += 3)
		{
			float derivative = gradient(phase.ptr<float>(i)[j + 1], phase.ptr<float>(i)[j])
							+ gradient(phase.ptr<float>(i + 1)[j + 1], phase.ptr<float>(i)[j + 1])
							+ gradient(phase.ptr<float>(i + 1)[j], phase.ptr<float>(i + 1)[j + 1])
							+ gradient(phase.ptr<float>(i)[j], phase.ptr<float>(i + 1)[j]);
			
			if (derivative > 0.01)	
			{
				residueMap->ptr<float>(i)[j] = positive;
				residueFlag Residue(Point(i, j), positive);
				residueList.insert(residueList.end(),Residue);
				//residueMap->ptr<float>(i)[b] = 0;
				//residueMap->ptr<float>(i)[b + 1] = 0;
				//residueMap->ptr<float>(i)[b + 2] = 255;
			}
			else if (derivative < -0.01)
			{
				residueMap->ptr<float>(i)[j] = negative;
				residueFlag Residue(Point(i, j), negative);
				residueList.insert(residueList.end(), Residue);
				//residueMap->ptr<float>(i)[b] = 255;
				//residueMap->ptr<float>(i)[b + 1] = 255;
				//residueMap->ptr<float>(i)[b + 2] = 255;
			}
			else
			{
				residueMap->ptr<float>(i)[j] = normal;
				//residueMap->ptr<float>(i)[b] = 0;
				//residueMap->ptr<float>(i)[b + 1] = 0;
				//residueMap->ptr<float>(i)[b + 2] = 0;
			}
		}


	return residueList;
}