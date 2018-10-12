#include<iostream>
#include <opencv2\opencv.hpp>
#include "FTP.h"
#include "QGPU.h"
#include "QualityMap.h"

using namespace std;
using namespace cv;

//�����У��������ξ���
Mat intrinsic, distortion;

//��ͫ����ο�ƽ��߶�L������ͫ�ľ���d
//const float L = 80;
const float L = 55;
//const float d = 15;
const float d = 9.5;
//�ο�����
Mat referenceDistort, reference, ref_FT;
//���ƺ�����
Mat objectDistort, object, object_FT;
//��任��ĸ�����
Mat referenceIFT, objectIFT;
//����ͼ
Mat hanning, qualityMap;
//��λ��
Mat phase_;
//�߶�
Mat height;

int main()
{
	readCalibrationFile(intrinsic, distortion);

	//��ȡ�ο���ͱ��ι�դͼ����תΪ�Ҷ�ͼ
	referenceDistort = imread("reference.jpg", 0);
	objectDistort = imread("head2.jpg", 0);

	if (!referenceDistort.data || !objectDistort.data)
	{
		cout << "ͼƬ��ȡʧ��" << endl;
		return -1;
	}
	else
		cout << "��ȡ�ɹ�" << endl;

	//У�������
	undistort(referenceDistort, reference, intrinsic, distortion);
	undistort(objectDistort, object, intrinsic, distortion);
	cout << "У�����" << endl;
	
	//�ü�����Ч����
	//ȫ������
	//reference = reference(Rect(250, 40, 300, 290));
	//object = object(Rect(250, 40, 300, 290));
	//��������
	reference = reference(Rect(330, 40, 150, 150));
	object = object(Rect(330, 40, 150, 150));
	//imshow("reference_0", reference);
	imshow("object_0", object);
	
	//����������ͼ������Ҷ�任
	FT(reference, &ref_FT);
	FT(object, &object_FT);
	
	//�γ�����ͼ
	object_FT.copyTo(hanning);
	hamingFilter(&hanning, &qualityMap);

	//�˲��˳���Ƶ����
	filter(&ref_FT);
	filter(&object_FT);
	
	//�渵��Ҷ�任,��û��DFT_REAL_OUTPUT��־λ���������Ϊ��������
	//idft(ref_FT, referenceIFT, DFT_REAL_OUTPUT | DFT_SCALE); 
	//idft(object_FT, objectIFT, DFT_REAL_OUTPUT | DFT_SCALE);
	//idft(ref_FT, referenceIFT, DFT_SCALE);
	//idft(object_FT, objectIFT, DFT_SCALE);
	IFT(ref_FT, &referenceIFT);
	IFT(object_FT, &objectIFT);
	
	/*
	//cv::DftFlags
	//��ʾ��任���ͼƬ
	normalize(referenceIFT, referenceIFT, 0, 1, NORM_MINMAX);//��ʾ��Ҫ���й�һ��
	normalize(objectIFT, objectIFT, 0, 1, NORM_MINMAX);
	flip(objectIFT, objectIFT, -1);
	imshow("reference", referenceIFT);
	imshow("object", objectIFT);
	*/
	
	
	//����λ��
	phase_ = dstPhase(referenceIFT, objectIFT, &qualityMap);
	
	//��߶�
	height = getHeight(L, d, phase_);

	//д��txt�ļ�
	write_txt("height.txt", height);
	//cout << "Writing finished" << endl;
	normalize(height, height, 0, 1, NORM_MINMAX);
	imshow("height", height);

	while (1)
	{
		if (waitKey(30) == 27)
		{
			cout << "End" << endl;
			destroyAllWindows();
			break;
		}
			
	}
	
	
	return 0;
}
