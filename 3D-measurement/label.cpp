
/*È±ÏÝ±ê×¢*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <fstream>
#include <vector>
#include<opencv2/opencv.hpp>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<int> indicestotal;

int num = 0;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;

	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
		indicestotal.push_back(indices[i]);
		
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(clicked_points_3d, 255, 0, 0);

	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";

	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
}

void main()
{
	int ti = 3;
	while (ti < 12)
	{
		std::ostringstream str_groupfilename;
		str_groupfilename << "./5555555/modified/" << ti << "_organized_5555555.pcd";
		//std::string filename("0_organized_bigdefect.pcd");
		if (pcl::io::loadPCDFile(str_groupfilename.str(), *cloud))
		{
			std::cerr << "ERROR: Cannot open file " << std::endl;
			return;
		}
		//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(cloud, "rgb");
		//my_viewer2.addPointCloud(ptrcloud_original, rgb, "sample cloud");
		int height = cloud->height;
		int width = cloud->width;
		viewer->setBackgroundColor(255, 255, 255);
		//viewer->setCameraPosition(0, 0, 10, 0, 0, -1, 0, 0, 1);
		viewer->addPointCloud(cloud, "sample cloud");
		//viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
		viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		/*for (int i = 0; i < indicestotal.size(); i++)
		{
			cloud->points[indicestotal[i]].z = 0;
		}
		pcl::visualization::PCLVisualizer my_viewer2;
		my_viewer2.addPointCloud(cloud);
		while (!my_viewer2.wasStopped())
		{
			my_viewer2.spinOnce(100);
		}*/
		ofstream fout; 
		std::ostringstream gtfilename;
		gtfilename << "./5555555/modified/" << ti << "_gt.txt";
		fout.open(gtfilename.str());
		cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
		for (int i = 0; i < indicestotal.size(); i++)
		{
			fout << indicestotal[i] << endl;
			//		int row = height-1-floor(indicestotal[i] / width);
			int row = floor(indicestotal[i] / width);
			int col = indicestotal[i] % width;
			uchar* ptr = img.ptr<uchar>(row);
			for (int j = col * img.channels(); j < (col + 1)*img.channels(); j++)
			{
				ptr[j] = 255;
			}

			//img.at<cv::Vec3b>(row, col)[2] = cv::Vec3b(255, 255, 255);
		}
		fout.close();

		std::ostringstream filename;
		filename << "./5555555/modified/" << ti << "_gt.png";
		cv::imwrite(filename.str(), img);
		cv::imshow("abc", img);
		cv::waitKey(10000);
		cout << ti << endl;
		ti++;
	}
	
	
}
