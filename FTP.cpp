#include "FTP.h"

/*��ȡ�����У��������ξ���
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

/*��ͼ������ɢ����Ҷ�任
**���룺��任��ͼ��
**���������Ҷ�任��ĸ�������
*/
void FT(Mat contour0, Mat *contour)
{
	//���ͼ���ǻҶ�ͼ��תΪ��ͨ���Ҷ�ͼ
	//if (contour0.depth() != 1)
	//	cvtColor(contour0, contour0, COLOR_BGR2GRAY);
	
	//��ͼ�������ʺ�DFT�����ųߴ��С
	int rows = getOptimalDFTSize(contour0.rows);//�������ųߴ�
	int cols = getOptimalDFTSize(contour0.cols);
	
	Mat FT;
	copyMakeBorder(contour0, FT, 0, rows - contour0.rows, 0, cols - contour0.cols, BORDER_CONSTANT, Scalar::all(0));

	//Ϊ����Ҷ�任�Ľ����ʵ�����鲿������洢�ռ�
	//��planes������Ϻϲ���һ��˫ͨ��������complex
	FT.convertTo(FT, CV_32F);
	Mat planes[] = { Mat::zeros(FT.size(), CV_32F) , Mat::zeros(FT.size(), CV_32F) };
	merge(planes, 2, *contour);

	//��ɢ����Ҷ�任
	dft(FT, *contour, DFT_COMPLEX_OUTPUT);
	
	//�޳�����ͼ������ӵ�����
	*contour = (*contour)(Rect(0, 0, (*contour).cols & -2, (*contour).rows & -2));
	
	cout << "����Ҷ�任���" << endl;

	return;
}

/*******************************************************/
/*�Ը���Ҷ�任���ͼ���˲����˳���Ƶ����
**���룺��������
**������������Ƶ����f������ֵ������ͨ�˲�
*/
void filter(Mat *contour)
{
	//�����˲���
	Mat filter = Mat::zeros((*contour).size(), CV_32F);
	
	//ȫ������
	//rectangle(filter, Rect(50, 0, 25, 15), 1, -1, 8);
	//rectangle(filter, Rect(50, filter.cols - 16, 25, 15), 1, -1, 8);
	//��������
	rectangle(filter, Rect(25, 0, 15, 10), 1, -1, 8);
	rectangle(filter, Rect(25, filter.cols - 16, 15, 10), 1, -1, 8);
	//imshow("�˲���", filter);

	//������Ҷ�任�󸴺Ͼ����Ϊ������ͨ������
	Mat planes[] = { Mat::zeros((*contour).size(), CV_32F) , Mat::zeros((*contour).size(), CV_32F) };
	split((*contour), planes);

	/*
	//���Ƶ����
	Mat spectrum;
	magnitude(planes[0], planes[1], spectrum);
	write_txt("spectrum.txt", spectrum);
	*/
	

	//�˲����븵��Ҷ�任�������ˣ�ֻ������Ƶ����
	planes[0] = planes[0].mul(filter);
	planes[1] = planes[1].mul(filter);
	//ȡ���Ϊ��任��׼��
	planes[1] = planes[1].mul(-1);
	
	//���ºϳɸ��Ͼ���
	merge(planes, 2, (*contour));

	cout << "�˲����" << endl;

	return;
}


/*******************************************************/
/*��ɢ����Ҷ��任
**���룺����Ҷ�任��ĸ�����
**���������Ҷ��任��ĸ�������
*/
void IFT(Mat contourFT, Mat *contourIFT)
{
	//�����任����任
	dft(contourFT, *contourIFT, DFT_SCALE);

	//�������ȡ����
	Mat planes[] = { Mat::zeros((*contourIFT).size(), CV_32F) , Mat::zeros((*contourIFT).size(), CV_32F) };
	split(*contourIFT, planes);
	planes[1].mul(-1);

	merge(planes, 2, *contourIFT);
	//blur(*contourIFT, *contourIFT, Size(3, 3));
	cout << "��任���" << endl;
	return;
}


/*******************************************************/
/*����λ��
**���룺�ο�ƽ���Ŀ��ͼ��ĸ�������
**�������ֱ����������������λ�������������
*/
Mat dstPhase(Mat ref, Mat obj, Mat *qualityMap)
{
	//��ref��obj���������Ϊʵ�����鲿��ʽ
	Mat planes_ref[] = { Mat::zeros(ref.size(), CV_32F) , Mat::zeros(ref.size(), CV_32F) };
	split(ref, planes_ref);
	Mat planes_obj[] = { Mat::zeros(ref.size(), CV_32F) , Mat::zeros(obj.size(), CV_32F) };
	split(obj, planes_obj);
	
	
	//�ֱ�洢�����������λ
	Mat phase_ref = Mat::zeros(ref.size(), CV_32F);
	Mat phase_obj = Mat::zeros(obj.size(), CV_32F);

	//���������������λ
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
	
	cout << "����λ���" << endl;
	return phase;
}


/*******************************************************/
/*�������λ
**���룺��λ
**������ȡ����������λ�Ƚϣ���ֵ����pi����һ���ۼ�2pi����ֵС��pi�����-pi���䣻��ֵС��-pi����һ���-2pi
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
/*��߶�
**���룺��λ����
**������h=l*phase/(phase-2*pi*f*d)
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

	cout << "�߶ȼ������" << endl;
	return height;
}


/*******************************************************/
/*������д��txt�ļ�
**���룺�ļ���������
**������д��txt�ļ�
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