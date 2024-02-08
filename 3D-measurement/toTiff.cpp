

/*提取xyz三通道转化为tiff文件*/
#include <algorithm>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <string>
#include "tiffio.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include<opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>
////#include <pcl/filters/passthrough.h>
using namespace pcl;
using namespace std;

void pointcloud2tiff(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string filename)
{

	tiff *out = TIFFOpen(filename.c_str(), "w");
	int sampleperpixel = 3; // x, y, z
	int bitspersample = 32; //float
	int width = cloud->width;
	int height = cloud->height;
	int num = width * height*sampleperpixel;
	float *image = new float[num];
	//uint16 *image = new uint16[num];
	int linesamples = sampleperpixel * width;
	int linebytes = linesamples * sizeof(float);
	int i = 0;
	while(i< num)
	{
		for (int r = 0; r < height; ++r) {
			for (int c = 0; c < width; ++c) {
				
				image[i++] = cloud->points[c + r * width].x;
				image[i++] = cloud->points[c + r * width].y;
				image[i++] = cloud->points[c + r * width].z;
				
			}
		}

	}
	

	
	// set header
	TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);                   // set the width of the image
	TIFFSetField(out, TIFFTAG_IMAGELENGTH, height);                 // set the height of the image
	TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, sampleperpixel);     // set number of channels per pixel
	TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, bitspersample);        // set the size of the channels
	TIFFSetField(out, TIFFTAG_ORIENTATION, orientation_topleft);    // set the origin of the image.
																	// some other essential fields to set that you do not have to understand for now.
	TIFFSetField(out, TIFFTAG_PLANARCONFIG, planarconfig_contig);
	TIFFSetField(out, TIFFTAG_PHOTOMETRIC, photometric_rgb);
	TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, sampleformat_ieeefp);

	unsigned char  *buf = NULL;
	if (TIFFScanlineSize(out)) {
		buf = (unsigned char *)_TIFFmalloc(linebytes);
	}
	else {
		buf = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(out));
	}

	// we set the strip size of the file to be size of one row of pixels
	TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(out, width*sampleperpixel));

	
	for (int row = 0; row < height; row++) {
		//memcpy(buf, &image[(height - row - 1)*linebytes], linebytes * sizeof(float));
		memcpy(buf, &image[row*linesamples], linebytes);
		if (TIFFWriteScanline(out, buf, row, 0) < 0) {
			fprintf(stderr, "scanline %d: write error.\n", row);
			break;
		}
	}
	//for (int row = height-1; row >= 0; row--) {
	//	//memcpy(buf, &image[(height - row - 1)*linebytes], linebytes * sizeof(float));
	//	memcpy(buf, &image[row*linesamples], linebytes);
	//	if (tiffwritescanline(out, buf, height-1-row, 0) < 0) {
	//		fprintf(stderr, "scanline %d: write error.\n", row);
	//		break;
	//	}
	//}

	TIFFClose(out);
	if (buf) {
		_TIFFfree(buf);
	}

}



int main()
{
	int ti = 0;
	while (ti <1)
	{

		std::ostringstream str_groupfilename;
		str_groupfilename << "./0/"<<ti<<".pcd";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPCDFile(str_groupfilename.str(), *cloud);
		
		
	/*	pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb(cloud, "rgb");
		viewer.setBackgroundColor(255, 255, 255);
		viewer.addPointCloud(cloud_visual, "sample cloud");
		//viewer.addpointcloud(cloudcopy);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}*/


		std::ostringstream filename;
		filename << ti << ".tiff";
		pointcloud2tiff(cloud, filename.str());


		/*dst = src.clone();
		int rows = src.rows;
		int cols = src.cols;
		for (int row = 0; row < rows; row++) 
		{
			for (int col = 0; col < cols; col++) 
			{
				float val = src.at<uchar>(row, col);
				dst.at<uchar>(rows - row - 1, col)= val;
				cout << col << " ";
			}
			cout <<endl;
		}*/
		/*cv::imshow("dst", dst);
		cv::waitKey(10000);*/

		//cv::mat img(height, width, cv_8uc3, cv::scalar(0, 0, 0));
		//for (int i = 0; i < cloud->size(); i++)
		//{
		//	//fout << indicestotal[i] << endl;
		//	int row = height-1-floor(i / width);
		//	//int row = floor(i / width);
		//	int col = i % width;
		//	uchar* ptr = img.ptr<uchar>(row);
		//	for (int j = col * img.channels(); j < (col + 1)*img.channels(); j++)
		//	{
		//		ptr[j] = cloud->points[i].r;
		//	}
		//

		//}
		////fout.close();
		
		/*cv::imshow("abc", img);
		cv::waitkey(10000);*/
		cout << ti << endl;
		ti++;
	}

	return 0;
}

