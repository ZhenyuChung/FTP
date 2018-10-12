#include "QGPU.h"

/*******************************************************/
/*质量图指导解包裹
**输入：包裹相位，quality map
**操作：根据质量图解包裹
**输出：解包裹相位
*/
void guidingUnwrap(Mat *phase, Mat *qualityMap)
{
	Mat mask = Mat::zeros((*phase).size(), CV_32F);//判断是否已解包裹的mask
	Mat n = Mat::zeros((*phase).size(), CV_32F);//记录解包裹累加的次数n
	
	Point seed;
	minMaxLoc(*qualityMap, 0, 0, 0, &seed);//找到quality map中的最大值作为seed
	mask.ptr<float>(seed.x)[seed.y] = Unwrapped;

	//生成最大堆，每次弹出quality map中最大值用于解包裹
	priority_queue<Node_> guiding;
	push(phase, &guiding, &seed, qualityMap, &mask, &n);//将seed点的4个邻点进队并解包裹

	while (!guiding.empty())//队列非空则循环
	{
		Node_ unwrappingNode = guiding.top();
		guiding.pop();//队首出队
		push(phase, &guiding, &unwrappingNode.point, qualityMap, &mask, &n);//将最大点的未进队的邻点进队并解包裹
		//imshow("mask", mask);
		//waitKey(1);
	}
	cout << "guiding unwrap finished" << endl;
	
	return;
}


/*******************************************************/
/*将一点的四个邻点进队并解包裹
**输入：队列，要进队的一点,quality map，mask
**操作：将该点的四个邻点进队并解包裹
**
*/
void push(Mat *phase, priority_queue<Node_> *guiding, Point *point, Mat *qualityMap, Mat *mask, Mat *n)
{
	if ((point->x - 1) >= 0)//上邻点,检查边界
	{
		if (mask->ptr<float>(point->x - 1)[point->y] == Wrapped)
		{
			Node_ up(Point(point->x - 1, point->y), qualityMap->ptr<float>(point->x - 1)[point->y]);
			guiding->push(up);
			unwrap(phase, mask, *point, &(up.point), n);//解包裹
			//InSARunwrap(phase, mask, &(up.point));
		}
	}
	
	if ((point->x + 1) < qualityMap->rows)//下邻点,检查边界
	{
		if (mask->ptr<float>(point->x + 1)[point->y] == Wrapped)
		{
			Node_ down(Point(point->x + 1, point->y), qualityMap->ptr<float>(point->x + 1)[point->y]);
			guiding->push(down);
			unwrap(phase, mask, *point, &(down.point), n);//解包裹
			//InSARunwrap(phase, mask, &(down.point));
		}
	}
	
	if ((point->y - 1) >= 0)//左邻点,检查边界
	{
		if (mask->ptr<float>(point->x)[point->y - 1] == Wrapped)
		{
			Node_ left(Point(point->x, point->y - 1), qualityMap->ptr<float>(point->x)[point->y - 1]);
			guiding->push(left);
			unwrap(phase, mask, *point, &(left.point), n);//解包裹
			//InSARunwrap(phase, mask, &(left.point));
		}
	}
	
	if ((point->y + 1) < qualityMap->cols)//右邻点,检查边界
	{
		if (mask->ptr<float>(point->x)[point->y + 1] == Wrapped)
		{
			Node_ right(Point(point->x, point->y + 1), qualityMap->ptr<float>(point->x)[point->y + 1]);
			guiding->push(right);
			unwrap(phase, mask, *point, &(right.point), n);//解包裹
			//InSARunwrap(phase, mask, &(right.point));
		}
	}
	
	return;
}


/*******************************************************/
/*将一点解包裹
**输入：包裹相位，mask，点
**操作：将该点解包裹
**
*/
void unwrap(Mat *phase, Mat *mask, Point seed, Point *point, Mat *n)
{
	
	float diff = phase->ptr<float>(point->x)[point->y] + 2 * pi * n->ptr<float>(seed.x)[seed.y] - phase->ptr<float>(seed.x)[seed.y];

	if (diff > pi)
	{
		n->ptr<float>(point->x)[point->y] = n->ptr<float>(seed.x)[seed.y] - 1;
		phase->ptr<float>(point->x)[point->y] += 2 * pi * n->ptr<float>(point->x)[point->y];
	}

	else if ((diff <= pi) && (diff >= -pi))
	{
		n->ptr<float>(point->x)[point->y] = n->ptr<float>(seed.x)[seed.y];
		phase->ptr<float>(point->x)[point->y] += 2 * pi * n->ptr<float>(point->x)[point->y];
	}

	else if (diff < -pi)
	{
		n->ptr<float>(point->x)[point->y] = n->ptr<float>(seed.x)[seed.y] + 1;
		phase->ptr<float>(point->x)[point->y] += 2 * pi * n->ptr<float>(point->x)[point->y];
	}

	/*
	float diff = phase->ptr<float>(seed.x)[seed.y] - phase->ptr<float>(point->x)[point->y];
	int n = (diff /  pi) > 0.0 ? (diff + 0.5) : (diff - 0.5);
	phase->ptr<float>(point->x)[point->y] += (2 * n * pi);
	*/
	mask->ptr<float>(point->x)[point->y] = Unwrapped;//该点在mask上标记为Unwrapped

	return;
}


/*******************************************************/
/*InSAR解包裹
**输入：相位矩阵，mask，需要解包裹的点坐标
**
**操作：解包裹
**参考文献：Xu, Wei;Cumming, Ian. A region-growing algorithm for InSAR phase unwrapping[J].IEEE Transactions on Geoscience and Remote Sensing,1999,Vol.37(1): 124-134
*/
void InSARunwrap(Mat *phase, Mat *mask, Point *point)
{
	Mat fai = Mat::zeros(Size(1, 8), CV_32F);
	Mat omega = Mat::zeros(Size(1, 8), CV_32F);
	
	//参考论文公式（1）-（2）
	//左上fai[0,0]
	if (((point->x - 1) >= 0) && ((point->y - 1) >= 0))//边界
	{
		if (mask->ptr<float>(point->x - 1)[point->y - 1] == Unwrapped)//[x-1,y-1]
		{
			
			fai.ptr<float>(0)[0] = phase->ptr<float>(point->x - 1)[point->y - 1];
			omega.ptr<float>(0)[0] = 0.5;
		}

		if (((point->x - 2) >= 0) && ((point->y - 2) >= 0))//边界
		{
			if (mask->ptr<float>(point->x - 2)[point->y - 2] == Unwrapped)//[x-2,y-2]
			{
				fai.ptr<float>(0)[0] = fai.ptr<float>(0)[0] * 2 - phase->ptr<float>(point->x - 2)[point->y - 2];
				omega.ptr<float>(0)[0] = 1;
			}
		}
	}

	//左fai[1,0]
	if ((point->y - 1) >= 0)//边界
	{
		if (mask->ptr<float>(point->x)[point->y - 1] == Unwrapped)//[x,y-1]
		{
			fai.ptr<float>(1)[0] = phase->ptr<float>(point->x)[point->y - 1];
			omega.ptr<float>(1)[0] = 0.5;
		}

		if ((point->y - 2) >= 0)//边界
		{
			if (mask->ptr<float>(point->x)[point->y - 2] == Unwrapped)//[x,y-2]
			{
				fai.ptr<float>(1)[0] = fai.ptr<float>(1)[0] * 2 - phase->ptr<float>(point->x)[point->y - 2];
				omega.ptr<float>(1)[0] = 1;
			}
		}
	}

	//左下fai[2,0]
	if (((point->x + 1) < phase->rows) && ((point->y - 1) >= 0))//边界
	{
		if (mask->ptr<float>(point->x + 1)[point->y - 1] == Unwrapped)//[x+1,y-1]
		{
			fai.ptr<float>(2)[0] = phase->ptr<float>(point->x + 1)[point->y - 1];
			omega.ptr<float>(2)[0] = 0.5;
		}

		if (((point->x + 2) < phase->rows) && ((point->y - 2) >= 0))//边界
		{
			if (mask->ptr<float>(point->x + 2)[point->y - 2] == Unwrapped)//[x+2,y-2]
			{
				fai.ptr<float>(2)[0] = fai.ptr<float>(2)[0] * 2 - phase->ptr<float>(point->x + 2)[point->y - 2];
				omega.ptr<float>(2)[0] = 1;
			}
		}
	}

	//上fai[3,0]
	if ((point->x - 1) >= 0)//边界
	{
		if (mask->ptr<float>(point->x - 1)[point->y] == Unwrapped)//[x-1,y]
		{
			fai.ptr<float>(3)[0] = phase->ptr<float>(point->x - 1)[point->y];
			omega.ptr<float>(3)[0] = 0.5;
		}

		if ((point->x - 2) >= 0)//边界
		{
			if (mask->ptr<float>(point->x - 2)[point->y] == Unwrapped)//[x-2,y]
			{
				fai.ptr<float>(3)[0] = fai.ptr<float>(3)[0] * 2 - phase->ptr<float>(point->x - 2)[point->y];
				omega.ptr<float>(3)[0] = 1;
			}
		}
	}

	//下fai[4,0]
	if ((point->x + 1) < phase->rows)//边界
	{
		if (mask->ptr<float>(point->x + 1)[point->y] == Unwrapped)//[x+1,y]
		{
			fai.ptr<float>(4)[0] = phase->ptr<float>(point->x + 1)[point->y];
			omega.ptr<float>(4)[0] = 0.5;
		}

		if ((point->x + 2) < phase->rows)//边界
		{
			if (mask->ptr<float>(point->x + 2)[point->y] == Unwrapped)//[x+2,y]
			{
				fai.ptr<float>(4)[0] = fai.ptr<float>(4)[0] * 2 - phase->ptr<float>(point->x + 2)[point->y];
				omega.ptr<float>(4)[0] = 1;
			}
		}
	}

	//右上fai[5,0]
	if (((point->x - 1) >= 0) && ((point->y + 1) < phase->cols))//边界
	{
		if (mask->ptr<float>(point->x - 1)[point->y + 1] == Unwrapped)//[x-1,y+1]
		{
			fai.ptr<float>(5)[0] = phase->ptr<float>(point->x - 1)[point->y + 1];
			omega.ptr<float>(5)[0] = 0.5;
		}

		if (((point->x - 2) >= 0) && ((point->y + 2) < phase->cols))//边界
		{
			if (mask->ptr<float>(point->x - 2)[point->y + 2] == Unwrapped)//[x-2,y+2]
			{
				fai.ptr<float>(5)[0] = fai.ptr<float>(5)[0] * 2 - phase->ptr<float>(point->x - 2)[point->y + 2];
				omega.ptr<float>(5)[0] = 1;
			}
		}
	}

	//右fai[6,0]
	if ((point->y + 1) < phase->cols)//边界
	{
		if (mask->ptr<float>(point->x)[point->y + 1] == Unwrapped)//[x,y+1]
		{
			fai.ptr<float>(6)[0] = phase->ptr<float>(point->x)[point->y + 1];
			omega.ptr<float>(6)[0] = 0.5;
		}

		if ((point->y + 2) < phase->cols)//边界
		{
			if (mask->ptr<float>(point->x)[point->y + 2] == Unwrapped)//[x,y+2]
			{
				fai.ptr<float>(6)[0] = fai.ptr<float>(6)[0] * 2 - phase->ptr<float>(point->x)[point->y + 2];
				omega.ptr<float>(6)[0] = 1;
			}
		}
	}

	//右下fai[7,0]
	if (((point->x + 1) < phase->rows) && ((point->y + 1) < phase->cols))//边界
	{
		if (mask->ptr<float>(point->x + 1)[point->y + 1] == Unwrapped)//[x+1,y+1]
		{
			fai.ptr<float>(7)[0] = phase->ptr<float>(point->x + 1)[point->y + 1];
			omega.ptr<float>(7)[0] = 0.5;
		}

		if (((point->x + 2) < phase->rows) && ((point->y + 2) < phase->cols))//边界
		{
			if (mask->ptr<float>(point->x + 2)[point->y + 2] == Unwrapped)//[x+2,y+2]
			{
				fai.ptr<float>(7)[0] = fai.ptr<float>(7)[0] * 2 - phase->ptr<float>(point->x + 2)[point->y + 2];
				omega.ptr<float>(7)[0] = 1;
			}
		}
	}
	
	//参考论文公式(3)-(5)
	fai = fai.mul(omega); 
	float fai_p = sumMat(fai) / sumMat(omega);
	fai_p = (fai_p - phase->ptr<float>(point->x)[point->y]) / 2 / pi;
	
	int n = (fai_p > 0.0) ? (fai_p + 0.5) : (fai_p - 0.5); 
	phase->ptr<float>(point->x)[point->y] += (2 * pi * n);

	mask->ptr<float>(point->x)[point->y] = Unwrapped;//该点在mask上标记为Unwrapped

	return;
}



/*******************************************************/
/*Mat类型求和
**输入：矩阵
**
**输出：矩阵各元素的和
*/
float sumMat(Mat m)
{
	float sum = 0;
	for (int i = 0; i < m.rows; i++)
		for (int j = 0; j < m.cols; j++)
			sum += m.ptr<float>(i)[j];

	return sum;
}