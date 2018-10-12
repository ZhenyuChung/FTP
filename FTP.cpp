#include "FTP.h"

/*读取摄像机校正的内外参矩阵
**
**
*/
void readCalibrationFile(Mat &intrinsic, Mat &distortion)
{
	FileStorage fs("calibrationMatrix.yml", FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "File not exist" << endl;
		return;
	}

	fs["intrinsic"] >> intrinsic;
	fs["distortion"] >> distortion;
	cout << "Read file successfully" << endl;
	return;
}

/*对图像做离散傅里叶变换
**输入：许变换的图像
**输出：傅里叶变换后的复数矩阵
*/
void FT(Mat contour0, Mat *contour)
{
	//如果图像不是灰度图，转为单通道灰度图
	//if (contour0.depth() != 1)
	//	cvtColor(contour0, contour0, COLOR_BGR2GRAY);
	
	//将图像扩大到适合DFT的最优尺寸大小
	int rows = getOptimalDFTSize(contour0.rows);//返回最优尺寸
	int cols = getOptimalDFTSize(contour0.cols);
	
	Mat FT;
	copyMakeBorder(contour0, FT, 0, rows - contour0.rows, 0, cols - contour0.cols, BORDER_CONSTANT, Scalar::all(0));

	//为傅里叶变换的结果（实部与虚部）分配存储空间
	//将planes数组组合合并成一个双通道的数组complex
	FT.convertTo(FT, CV_32F);
	Mat planes[] = { Mat::zeros(FT.size(), CV_32F) , Mat::zeros(FT.size(), CV_32F) };
	merge(planes, 2, *contour);

	//离散傅里叶变换
	dft(FT, *contour, DFT_COMPLEX_OUTPUT);
	
	//剔除扩大图像新添加的像素
	*contour = (*contour)(Rect(0, 0, (*contour).cols & -2, (*contour).rows & -2));
	
	cout << "傅里叶变换完成" << endl;

	return;
}

/*******************************************************/
/*对傅里叶变换后的图像滤波，滤出基频分量
**输入：复数矩阵
**操作：保留基频分量f附近的值，即带通滤波
*/
void filter(Mat *contour)
{
	//建立滤波器
	Mat filter = Mat::zeros((*contour).size(), CV_32F);
	
	//全部条纹
	//rectangle(filter, Rect(50, 0, 25, 15), 1, -1, 8);
	//rectangle(filter, Rect(50, filter.cols - 16, 25, 15), 1, -1, 8);
	//脸部条纹
	rectangle(filter, Rect(25, 0, 15, 10), 1, -1, 8);
	rectangle(filter, Rect(25, filter.cols - 16, 15, 10), 1, -1, 8);
	//imshow("滤波器", filter);

	//将傅里叶变换后复合矩阵分为两个单通道矩阵
	Mat planes[] = { Mat::zeros((*contour).size(), CV_32F) , Mat::zeros((*contour).size(), CV_32F) };
	split((*contour), planes);

	/*
	//检查频谱用
	Mat spectrum;
	magnitude(planes[0], planes[1], spectrum);
	write_txt("spectrum.txt", spectrum);
	*/
	

	//滤波器与傅里叶变换后矩阵相乘，只保留基频分量
	planes[0] = planes[0].mul(filter);
	planes[1] = planes[1].mul(filter);
	//取共轭，为逆变换做准备
	planes[1] = planes[1].mul(-1);
	
	//重新合成复合矩阵
	merge(planes, 2, (*contour));

	cout << "滤波完成" << endl;

	return;
}


/*******************************************************/
/*离散傅里叶逆变换
**输入：傅里叶变换后的复矩阵
**输出：傅里叶逆变换后的复数矩阵
*/
void IFT(Mat contourFT, Mat *contourIFT)
{
	//用正变换做逆变换
	dft(contourFT, *contourIFT, DFT_SCALE);

	//计算后再取共轭
	Mat planes[] = { Mat::zeros((*contourIFT).size(), CV_32F) , Mat::zeros((*contourIFT).size(), CV_32F) };
	split(*contourIFT, planes);
	planes[1].mul(-1);

	merge(planes, 2, *contourIFT);
	//blur(*contourIFT, *contourIFT, Size(3, 3));
	cout << "逆变换完成" << endl;
	return;
}


/*******************************************************/
/*求相位差
**输入：参考平面和目标图像的复数矩阵
**操作：分别求两个复矩阵的相位，求反正切再相减
*/
Mat dstPhase(Mat ref, Mat obj, Mat *qualityMap)
{
	//将ref，obj复矩阵分离为实部与虚部形式
	Mat planes_ref[] = { Mat::zeros(ref.size(), CV_32F) , Mat::zeros(ref.size(), CV_32F) };
	split(ref, planes_ref);
	Mat planes_obj[] = { Mat::zeros(ref.size(), CV_32F) , Mat::zeros(obj.size(), CV_32F) };
	split(obj, planes_obj);
	
	
	//分别存储两个矩阵的相位
	Mat phase_ref = Mat::zeros(ref.size(), CV_32F);
	Mat phase_obj = Mat::zeros(obj.size(), CV_32F);

	//求两个复矩阵的相位
	for (int i = 0; i < phase_ref.rows; i++)
	{
		for (int j = 0; j < phase_ref.cols; j++)
		{
			phase_ref.ptr<float>(i)[j] = atan2f(planes_ref[1].ptr<float>(i)[j], planes_ref[0].ptr<float>(i)[j]);
			phase_obj.ptr<float>(i)[j] = atan2f(planes_obj[1].ptr<float>(i)[j], planes_obj[0].ptr<float>(i)[j]);
		}
	}
	//phase(planes_ref[0], planes_ref[1], phase_ref);
	//phase(planes_obj[0], planes_obj[1], phase_obj);
	
	Mat residueMap;
	vector<residueFlag> residueList(residue(phase_obj, &residueMap));
	//cout << residueMap << endl;
	//normalize(residueMap, residueMap, 0, 1, NORM_MINMAX);
	imshow("residue", residueMap); 
	
	//phaseDerivativeVariance(&phase_obj, qualityMap);
	//normalize(*qualityMap, *qualityMap, 0, 1, NORM_MINMAX);
	//imshow("quality map", *qualityMap);

	unwrappPhase(&phase_ref);
	write_txt("phase_ref.txt", phase_ref);
	
	clock_t start, end;
	start = clock();
	guidingUnwrap(&phase_obj, qualityMap);
	//unwrappPhase(&phase_obj);
	end = clock();
	cout << (double)(end - start) / CLOCKS_PER_SEC << endl;
	write_txt("phase_obj.txt", phase_obj);
	
	Mat phase = phase_obj - phase_ref;
	
	//for (int j = 0; j < phase.cols; j++)
	//	cout << phase.ptr<float>(80)[j] << endl;
	
	cout << "解相位完成" << endl;
	return phase;
}


/*******************************************************/
/*解包裹相位
**输入：相位
**操作：取相邻两点相位比较，差值大于pi，后一点累加2pi；差值小于pi或大于-pi不变；差值小于-pi，后一点加-2pi
*/
void unwrappPhase(Mat *phase)
{

	float *data;
	for (int i = 0; i < (*phase).rows; i++)
	{
		int n = 0;
		for (int j = 0; j < ((*phase).cols - 1); j++)
		{
			data = (*phase).ptr<float>(i);
			if ((data[j + 1] - data[j]) > pi)
			{
				if (((data[j + 1] - (pi * n * 2)) - data[j]) >= (pi))
					data[j + 1] -= (pi * (++n) * 2);
				else
					data[j + 1] -= (pi * n * 2);
			}
			
			else if((data[j + 1] - data[j]) < (-pi))
			{
				if (((data[j + 1] + (pi * n * 2)) - data[j]) <= (-pi))
					data[j + 1] += (pi * (++n) * 2);
				else
					data[j + 1] += (pi * n * 2);
			}
		}
	}

	return;
}



/*******************************************************/
/*求高度
**输入：相位矩阵
**操作：h=l*phase/(phase-2*pi*f*d)
*/
Mat getHeight(float L, float d, Mat phase)
{
	Mat height = Mat::zeros(phase.size(), CV_32F);
	float delta = 2 * d * pi /0.4;
	for (int i = 0; i < height.rows; i++)
	{
		for (int j = 0; j < height.cols; j++)
			height.ptr<float>(i)[j] = phase.ptr<float>(i)[j] * L /(phase.ptr<float>(i)[j] - delta);
			//height.ptr<float>(i)[j] = phase.ptr<float>(i)[j] * L*0.4 / (phase.ptr<float>(i)[j] * 0.4 / 2 / pi - d);
	}

	cout << "高度计算完成" << endl;
	return height;
}


/*******************************************************/
/*将矩阵写入txt文件
**输入：文件名，矩阵
**操作：写入txt文件
*/
void write_txt(char *name, Mat data)
{
	ofstream h;
	h.open(name, ios::trunc);

	for (int i = 0; i < data.rows; i++)
	{
		for (int j = 0; j < data.cols; j++)
		{
			h << data.ptr<float>(i)[j] << " ";
		}
		h << endl;
	}

	h.close();

	return;
}