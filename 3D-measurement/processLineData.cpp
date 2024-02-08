//处理线激光数据

#include <chrono>
#include <pcl/common/common.h>
//#include <pcl/filters/crop_box.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <conio.h>
#include <sstream>
using namespace pcl;
using namespace std;

int main()
{
	vector<string> vec_str;
	vector<vector<float>> vec_float;
	vector<float> z;
	vector<float> x;
	vector<float> y;
	float temp;
	ifstream infile("line.csv", ios::in);
	if (!infile)
	{
		cout << "打开文件失败！" << endl;
		exit(1);
	}
	//int count = 0;
	string line;
	string field;
	while (getline(infile, line))//按行读取csv文件中的数据
	{
		string field;
		istringstream sin(line); //将整行字符串line读入到字符串流sin中
		while (getline(sin, field, ','))
		{
			vec_str.push_back(field);
		}

	}
	infile.close();

	//string infile = "data1.csv";
	//vector<string> vec_str;
	//vector<vector<float>> vec_float;
	//vector<float> z;
	//vector<float> x;
	//vector<float> y;
	//float temp;
	//
	//ifstream input(infile);
	//if (input) // ensure that the input is seccessful
	//{
	//	string word;
	//	while (input >> word)
	//	{
	//		vec_str.push_back(word);
	//	}
	//}
	//else
	//{
	//	cerr << "cannot open file " << infile << "." << endl;
	//}
	//input.close();
	int count = 0;
	int ycount = 0;
	for (auto it = vec_str.begin(); it != vec_str.end(); it++)
	{
		//其作用是把字符串分解为单词
		istringstream istr(*it);
		string str;
		//以空格为界，把istringstream中数据取出放入到依次s中
		while (istr >> str)
		{
			//将string类型转换成float
			temp = atof(str.c_str());

		}
		count++;
		if (count <= 1847)
			x.push_back(temp);
		else if (count % 1847 == 1)
		{
			y.push_back(temp);
			if (z.size() > 0)
			{
				vec_float.push_back(z);
				ycount++;
				z.clear();
			}

		}
		else
			z.push_back(temp);
	}
	if (z.size() > 0)
		vec_float.push_back(z);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrcloud_original(new pcl::PointCloud<pcl::PointXYZ>);
	int num = y.size()*(x.size() - 1);
	ptrcloud_original->resize(num);
	count = 0;
	if (count < num)
	{
		for (int i = 0; i < vec_float.size(); i++)
		{
			for (int j = 0; j < vec_float[i].size(); j++)
			{
				ptrcloud_original->points[count].x = x[j + 1];
				ptrcloud_original->points[count].y = y[i];
				ptrcloud_original->points[count].z = vec_float[i][j];
				count++;
			}

		}
	}
	ptrcloud_original->width = x.size() - 1;
	ptrcloud_original->height = y.size();
	//ptrcloud_original->points.resize(num);
	ptrcloud_original->is_dense = false;
	std::ostringstream filename;
	filename << "data5.pcd";
	std::cout << "saving " << filename.str() << std::endl;

	bool success = true;
	success = -1 != pcl::io::savePCDFileBinary(filename.str(), *ptrcloud_original);

	if (!success)
	{
		std::cerr << "failed to save data5.pcd" << std::endl;
	}
	else
	{

		std::cout << "saving succeed data5.pcd" << std::endl;
	}

	return 0;
}
