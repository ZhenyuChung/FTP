#include "QGPU.h"

/*******************************************************/
/*����ͼָ�������
**���룺������λ��quality map
**��������������ͼ�����
**������������λ
*/
void guidingUnwrap(Mat *phase, Mat *qualityMap)
{
	Mat mask = Mat::zeros((*phase).size(), CV_32F);//�ж��Ƿ��ѽ������mask
	Mat n = Mat::zeros((*phase).size(), CV_32F);//��¼������ۼӵĴ���n
	
	Point seed;
	minMaxLoc(*qualityMap, 0, 0, 0, &seed);//�ҵ�quality map�е����ֵ��Ϊseed
	mask.ptr<float>(seed.x)[seed.y] = Unwrapped;

	//�������ѣ�ÿ�ε���quality map�����ֵ���ڽ����
	priority_queue<Node_> guiding;
	push(phase, &guiding, &seed, qualityMap, &mask, &n);//��seed���4���ڵ���Ӳ������

	while (!guiding.empty())//���зǿ���ѭ��
	{
		Node_ unwrappingNode = guiding.top();
		guiding.pop();//���׳���
		push(phase, &guiding, &unwrappingNode.point, qualityMap, &mask, &n);//�������δ���ӵ��ڵ���Ӳ������
		//imshow("mask", mask);
		//waitKey(1);
	}
	cout << "guiding unwrap finished" << endl;
	
	return;
}


/*******************************************************/
/*��һ����ĸ��ڵ���Ӳ������
**���룺���У�Ҫ���ӵ�һ��,quality map��mask
**���������õ���ĸ��ڵ���Ӳ������
**
*/
void push(Mat *phase, priority_queue<Node_> *guiding, Point *point, Mat *qualityMap, Mat *mask, Mat *n)
{
	if ((point->x - 1) >= 0)//���ڵ�,���߽�
	{
		if (mask->ptr<float>(point->x - 1)[point->y] == Wrapped)
		{
			Node_ up(Point(point->x - 1, point->y), qualityMap->ptr<float>(point->x - 1)[point->y]);
			guiding->push(up);
			unwrap(phase, mask, *point, &(up.point), n);//�����
			//InSARunwrap(phase, mask, &(up.point));
		}
	}
	
	if ((point->x + 1) < qualityMap->rows)//���ڵ�,���߽�
	{
		if (mask->ptr<float>(point->x + 1)[point->y] == Wrapped)
		{
			Node_ down(Point(point->x + 1, point->y), qualityMap->ptr<float>(point->x + 1)[point->y]);
			guiding->push(down);
			unwrap(phase, mask, *point, &(down.point), n);//�����
			//InSARunwrap(phase, mask, &(down.point));
		}
	}
	
	if ((point->y - 1) >= 0)//���ڵ�,���߽�
	{
		if (mask->ptr<float>(point->x)[point->y - 1] == Wrapped)
		{
			Node_ left(Point(point->x, point->y - 1), qualityMap->ptr<float>(point->x)[point->y - 1]);
			guiding->push(left);
			unwrap(phase, mask, *point, &(left.point), n);//�����
			//InSARunwrap(phase, mask, &(left.point));
		}
	}
	
	if ((point->y + 1) < qualityMap->cols)//���ڵ�,���߽�
	{
		if (mask->ptr<float>(point->x)[point->y + 1] == Wrapped)
		{
			Node_ right(Point(point->x, point->y + 1), qualityMap->ptr<float>(point->x)[point->y + 1]);
			guiding->push(right);
			unwrap(phase, mask, *point, &(right.point), n);//�����
			//InSARunwrap(phase, mask, &(right.point));
		}
	}
	
	return;
}


/*******************************************************/
/*��һ������
**���룺������λ��mask����
**���������õ�����
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
	mask->ptr<float>(point->x)[point->y] = Unwrapped;//�õ���mask�ϱ��ΪUnwrapped

	return;
}


/*******************************************************/
/*InSAR�����
**���룺��λ����mask����Ҫ������ĵ�����
**
**�����������
**�ο����ף�Xu, Wei;Cumming, Ian. A region-growing algorithm for InSAR phase unwrapping[J].IEEE Transactions on Geoscience and Remote Sensing,1999,Vol.37(1): 124-134
*/
void InSARunwrap(Mat *phase, Mat *mask, Point *point)
{
	Mat fai = Mat::zeros(Size(1, 8), CV_32F);
	Mat omega = Mat::zeros(Size(1, 8), CV_32F);
	
	//�ο����Ĺ�ʽ��1��-��2��
	//����fai[0,0]
	if (((point->x - 1) >= 0) && ((point->y - 1) >= 0))//�߽�
	{
		if (mask->ptr<float>(point->x - 1)[point->y - 1] == Unwrapped)//[x-1,y-1]
		{
			
			fai.ptr<float>(0)[0] = phase->ptr<float>(point->x - 1)[point->y - 1];
			omega.ptr<float>(0)[0] = 0.5;
		}

		if (((point->x - 2) >= 0) && ((point->y - 2) >= 0))//�߽�
		{
			if (mask->ptr<float>(point->x - 2)[point->y - 2] == Unwrapped)//[x-2,y-2]
			{
				fai.ptr<float>(0)[0] = fai.ptr<float>(0)[0] * 2 - phase->ptr<float>(point->x - 2)[point->y - 2];
				omega.ptr<float>(0)[0] = 1;
			}
		}
	}

	//��fai[1,0]
	if ((point->y - 1) >= 0)//�߽�
	{
		if (mask->ptr<float>(point->x)[point->y - 1] == Unwrapped)//[x,y-1]
		{
			fai.ptr<float>(1)[0] = phase->ptr<float>(point->x)[point->y - 1];
			omega.ptr<float>(1)[0] = 0.5;
		}

		if ((point->y - 2) >= 0)//�߽�
		{
			if (mask->ptr<float>(point->x)[point->y - 2] == Unwrapped)//[x,y-2]
			{
				fai.ptr<float>(1)[0] = fai.ptr<float>(1)[0] * 2 - phase->ptr<float>(point->x)[point->y - 2];
				omega.ptr<float>(1)[0] = 1;
			}
		}
	}

	//����fai[2,0]
	if (((point->x + 1) < phase->rows) && ((point->y - 1) >= 0))//�߽�
	{
		if (mask->ptr<float>(point->x + 1)[point->y - 1] == Unwrapped)//[x+1,y-1]
		{
			fai.ptr<float>(2)[0] = phase->ptr<float>(point->x + 1)[point->y - 1];
			omega.ptr<float>(2)[0] = 0.5;
		}

		if (((point->x + 2) < phase->rows) && ((point->y - 2) >= 0))//�߽�
		{
			if (mask->ptr<float>(point->x + 2)[point->y - 2] == Unwrapped)//[x+2,y-2]
			{
				fai.ptr<float>(2)[0] = fai.ptr<float>(2)[0] * 2 - phase->ptr<float>(point->x + 2)[point->y - 2];
				omega.ptr<float>(2)[0] = 1;
			}
		}
	}

	//��fai[3,0]
	if ((point->x - 1) >= 0)//�߽�
	{
		if (mask->ptr<float>(point->x - 1)[point->y] == Unwrapped)//[x-1,y]
		{
			fai.ptr<float>(3)[0] = phase->ptr<float>(point->x - 1)[point->y];
			omega.ptr<float>(3)[0] = 0.5;
		}

		if ((point->x - 2) >= 0)//�߽�
		{
			if (mask->ptr<float>(point->x - 2)[point->y] == Unwrapped)//[x-2,y]
			{
				fai.ptr<float>(3)[0] = fai.ptr<float>(3)[0] * 2 - phase->ptr<float>(point->x - 2)[point->y];
				omega.ptr<float>(3)[0] = 1;
			}
		}
	}

	//��fai[4,0]
	if ((point->x + 1) < phase->rows)//�߽�
	{
		if (mask->ptr<float>(point->x + 1)[point->y] == Unwrapped)//[x+1,y]
		{
			fai.ptr<float>(4)[0] = phase->ptr<float>(point->x + 1)[point->y];
			omega.ptr<float>(4)[0] = 0.5;
		}

		if ((point->x + 2) < phase->rows)//�߽�
		{
			if (mask->ptr<float>(point->x + 2)[point->y] == Unwrapped)//[x+2,y]
			{
				fai.ptr<float>(4)[0] = fai.ptr<float>(4)[0] * 2 - phase->ptr<float>(point->x + 2)[point->y];
				omega.ptr<float>(4)[0] = 1;
			}
		}
	}

	//����fai[5,0]
	if (((point->x - 1) >= 0) && ((point->y + 1) < phase->cols))//�߽�
	{
		if (mask->ptr<float>(point->x - 1)[point->y + 1] == Unwrapped)//[x-1,y+1]
		{
			fai.ptr<float>(5)[0] = phase->ptr<float>(point->x - 1)[point->y + 1];
			omega.ptr<float>(5)[0] = 0.5;
		}

		if (((point->x - 2) >= 0) && ((point->y + 2) < phase->cols))//�߽�
		{
			if (mask->ptr<float>(point->x - 2)[point->y + 2] == Unwrapped)//[x-2,y+2]
			{
				fai.ptr<float>(5)[0] = fai.ptr<float>(5)[0] * 2 - phase->ptr<float>(point->x - 2)[point->y + 2];
				omega.ptr<float>(5)[0] = 1;
			}
		}
	}

	//��fai[6,0]
	if ((point->y + 1) < phase->cols)//�߽�
	{
		if (mask->ptr<float>(point->x)[point->y + 1] == Unwrapped)//[x,y+1]
		{
			fai.ptr<float>(6)[0] = phase->ptr<float>(point->x)[point->y + 1];
			omega.ptr<float>(6)[0] = 0.5;
		}

		if ((point->y + 2) < phase->cols)//�߽�
		{
			if (mask->ptr<float>(point->x)[point->y + 2] == Unwrapped)//[x,y+2]
			{
				fai.ptr<float>(6)[0] = fai.ptr<float>(6)[0] * 2 - phase->ptr<float>(point->x)[point->y + 2];
				omega.ptr<float>(6)[0] = 1;
			}
		}
	}

	//����fai[7,0]
	if (((point->x + 1) < phase->rows) && ((point->y + 1) < phase->cols))//�߽�
	{
		if (mask->ptr<float>(point->x + 1)[point->y + 1] == Unwrapped)//[x+1,y+1]
		{
			fai.ptr<float>(7)[0] = phase->ptr<float>(point->x + 1)[point->y + 1];
			omega.ptr<float>(7)[0] = 0.5;
		}

		if (((point->x + 2) < phase->rows) && ((point->y + 2) < phase->cols))//�߽�
		{
			if (mask->ptr<float>(point->x + 2)[point->y + 2] == Unwrapped)//[x+2,y+2]
			{
				fai.ptr<float>(7)[0] = fai.ptr<float>(7)[0] * 2 - phase->ptr<float>(point->x + 2)[point->y + 2];
				omega.ptr<float>(7)[0] = 1;
			}
		}
	}
	
	//�ο����Ĺ�ʽ(3)-(5)
	fai = fai.mul(omega); 
	float fai_p = sumMat(fai) / sumMat(omega);
	fai_p = (fai_p - phase->ptr<float>(point->x)[point->y]) / 2 / pi;
	
	int n = (fai_p > 0.0) ? (fai_p + 0.5) : (fai_p - 0.5); 
	phase->ptr<float>(point->x)[point->y] += (2 * pi * n);

	mask->ptr<float>(point->x)[point->y] = Unwrapped;//�õ���mask�ϱ��ΪUnwrapped

	return;
}



/*******************************************************/
/*Mat�������
**���룺����
**
**����������Ԫ�صĺ�
*/
float sumMat(Mat m)
{
	float sum = 0;
	for (int i = 0; i < m.rows; i++)
		for (int j = 0; j < m.cols; j++)
			sum += m.ptr<float>(i)[j];

	return sum;
}