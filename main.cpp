#include<iostream>
#include <opencv2\opencv.hpp>
#include "FTP.h"
#include "QGPU.h"
#include "QualityMap.h"

using namespace std;
using namespace cv;

//摄像机校正的内外参矩阵
Mat intrinsic, distortion;

//出瞳距离参考平面高度L，两出瞳的距离d
//const float L = 80;
const float L = 55;
//const float d = 15;
const float d = 9.5;
//参考条纹
Mat referenceDistort, reference, ref_FT;
//调制后条纹
Mat objectDistort, object, object_FT;
//逆变换后的复矩阵
Mat referenceIFT, objectIFT;
//质量图
Mat hanning, qualityMap;
//相位差
Mat phase_;
//高度
Mat height;

int main()
{
	readCalibrationFile(intrinsic, distortion);

	//读取参考面和变形光栅图，并转为灰度图
	referenceDistort = imread("reference.jpg", 0);
	objectDistort = imread("head2.jpg", 0);

	if (!referenceDistort.data || !objectDistort.data)
	{
		cout << "图片读取失败" << endl;
		return -1;
	}
	else
		cout << "读取成功" << endl;

	//校正摄像机
	undistort(referenceDistort, reference, intrinsic, distortion);
	undistort(objectDistort, object, intrinsic, distortion);
	cout << "校正完成" << endl;
	
	//裁剪出有效部分
	//全部条纹
	//reference = reference(Rect(250, 40, 300, 290));
	//object = object(Rect(250, 40, 300, 290));
	//脸部条纹
	reference = reference(Rect(330, 40, 150, 150));
	object = object(Rect(330, 40, 150, 150));
	//imshow("reference_0", reference);
	imshow("object_0", object);
	
	//对两张条纹图做傅里叶变换
	FT(reference, &ref_FT);
	FT(object, &object_FT);
	
	//形成质量图
	object_FT.copyTo(hanning);
	hamingFilter(&hanning, &qualityMap);

	//滤波滤出基频分量
	filter(&ref_FT);
	filter(&object_FT);
	
	//逆傅里叶变换,如没有DFT_REAL_OUTPUT标志位，输出矩阵为复数矩阵
	//idft(ref_FT, referenceIFT, DFT_REAL_OUTPUT | DFT_SCALE); 
	//idft(object_FT, objectIFT, DFT_REAL_OUTPUT | DFT_SCALE);
	//idft(ref_FT, referenceIFT, DFT_SCALE);
	//idft(object_FT, objectIFT, DFT_SCALE);
	IFT(ref_FT, &referenceIFT);
	IFT(object_FT, &objectIFT);
	
	/*
	//cv::DftFlags
	//显示逆变换后的图片
	normalize(referenceIFT, referenceIFT, 0, 1, NORM_MINMAX);//显示需要进行归一化
	normalize(objectIFT, objectIFT, 0, 1, NORM_MINMAX);
	flip(objectIFT, objectIFT, -1);
	imshow("reference", referenceIFT);
	imshow("object", objectIFT);
	*/
	
	
	//求相位差
	phase_ = dstPhase(referenceIFT, objectIFT, &qualityMap);
	
	//求高度
	height = getHeight(L, d, phase_);

	//写入txt文件
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
