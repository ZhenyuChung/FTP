#include "BranchCut.h"

/*******************************************************/
/*Branch cut����residues
**���룺residue Map
**������
**�����
**reference:
**[1]Richard M. Goldstein;Howard A. Zebker;Charles L. Werner.Satellite radar interferometry: Two-dimensional phase unwrapping[J].Radio Science,1988,Vol.23: 713-720
**[2]D Ghiglia, M Pritt.Two Dimensional Phase Unwrapping: Theory, Algorithms and Software[J].
*/
void BranchCut(Mat residueMap, vector<residueFlag> residueList, Mat *Branch)
{
	*Branch = Mat::zeros(residueMap.size(), CV_16U);//�洢Branch

	//vector<vector<Point>> branch;
	
	for (int n = 0; n < residueList.size(); n++)
	{
		
		if (!residueList[n].activityOfResidue())
			continue;
		else
		{
			int charge = residueList[n].chargeVal;//balance the residue

			vector<Point> neighbourResidue;
			for (int boxSize = 3; charge != 0; boxSize += 2) //ѭ��ֱ��ƽ��
			{
				if (searchResidue(residueList[n].residuePoint, boxSize, &neighbourResidue))
				{
					for (int i = 0; i < neighbourResidue.size(); i++)
					{
						placeBranch(residueList[n].residuePoint, neighbourResidue[i], Branch);

					}
				}
				
				
				
			}
			
		}
	}
}