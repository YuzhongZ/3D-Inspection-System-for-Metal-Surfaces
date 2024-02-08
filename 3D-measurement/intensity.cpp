
#include <iostream>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp> 

using namespace std;
using namespace cv;

int OtsuAlgThreshold(const Mat image)
{
	if (image.channels() != 1)
	{
		cout << "Please input Gray-image!" << endl;
		return 0;
	}
	int T = 0;            //阈值
	double varValue = 0;  
	double w0 = 0;        
	double w1 = 0;        
	double u0 = 0;        
	double u1 = 0;        
	double Histogram[256] = { 0 };
	//int Histogram1[256] = { 0 };

	uchar *data = image.data;
	double totalNum = image.rows*image.cols;
	//灰度直方图分布
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			Histogram[data[i*image.step + j]]++;
			//Histogram1[data[i*image.step + j]]++;
		}
	}


	for (int i = 0; i < 255; i++)
	{

		w1 = 0;		u1 = 0;		w0 = 0;		u0 = 0;
		//***********背景计算*********
		for (int j = 0; j <= i; j++)
		{
			w1 += Histogram[j];     
			u1 += j * Histogram[j]; 
		}
		if (w1 == 0)
		{
			continue;
		}
		u1 = u1 / w1;              
		w1 = w1 / totalNum;        


		//***********前景计算*********
		for (int k = i + 1; k < 255; k++)
		{
			w0 += Histogram[k];     
			u0 += k * Histogram[k];
		}
		if (w0 == 0)
		{
			break;
		}
		u0 = u0 / w0;               
		w0 = w0 / totalNum;        


		//***********类间计算*********
		double varValueI = w0 * w1*(u1 - u0)*(u1 - u0);
		if (varValue < varValueI)
		{
			varValue = varValueI;
			T = i;
		}
	}

	//画以T为阈值的分割线
	//line(image1, Point(T, 235), Point(T, 0), Scalar(0, 0, 255), 2, 8);
	//imshow("直方图", image1);

	return T;
}

