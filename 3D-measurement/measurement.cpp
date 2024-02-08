#include <chrono>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>

#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp> 

//#include <pcl/search/search.h>
////#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d.h>


////#include <pcl/filters/filter_indices.h>
////#include <pcl/segmentation/region_growing.h>

//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>

//#include <pcl/filters/radius_outlier_removal.h>
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//#include <pcl/features/boundary.h>

//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/features/principal_curvatures.h>
//#include <pcl/features/moment_of_inertia_estimation.h>
//#include <pcl/filters/crop_hull.h>
//#include <pcl/surface/concave_hull.h>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/segmentation/region_growing.h>
//#include <pcl/filters/convolution_3d.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/features/don.h>
//#include <pcl/segmentation/impl/extract_clusters.hpp>
//#include <random>
//#include <time.h>

#include "function.h"
using namespace pcl;
using namespace std;
using namespace cv;

//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//boost::mutex cloud_mutex;
//pcl::visualization::PCLPlotter plotter;
//
//pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

float RadicalInverse_VdC(uint32_t bits)
{

	bits = (bits << 16u) | (bits >> 16u);
	bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
	bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
	bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
	bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
	return float(bits) * 2.3283064365386963e-10; // / 0x100000000
}



double make_hammersley_sequence(int index, int base) {
	double f = 1, r = 0;
	while (index > 0) {
		f = f / base;
		r = r + f * (index % base);
		index = index / base;
	}
	return r;
}




int main()
{

	int ti = 0;

	/*采样序列*/
	int sample_count = 64;
	vector<Eigen::Vector2f> u_v;
	for (uint32_t i = 0; i < sample_count; ++i)
	{
		u_v.push_back(Eigen::Vector2f(float(i) / float(sample_count), RadicalInverse_VdC(i)));
		//u_v.push_back(Eigen::Vector2f(float(i) / float(sample_count), make_hammersley_sequence(i, 2)));
	}


	ofstream fout;
	fout.open("2222222_sut.txt");

	while (ti < 20)
	{
		std::ostringstream str_groupfilename;
		str_groupfilename << "./2222222/" << ti << ".pcd";


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_original(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile(str_groupfilename.str(), *color_original) == -1)
		{
			std::cerr << "COULD NOT READ FILE " << str_groupfilename.str() << std::endl;
			system("pause");
			return (-1);
		}
		std::cout << "original color points size is:" << color_original->points.size() << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrcloud_original(new pcl::PointCloud<pcl::PointXYZ>);
		ptrcloud_original->resize(color_original->size());

		auto t1 = chrono::steady_clock::now();

		cv::Mat image(color_original->height, color_original->width, CV_8UC3, cv::Scalar(0, 0, 0));
		for (int i = 0; i < color_original->size(); i++)
		{
			ptrcloud_original->points[i].x = color_original->points[i].x;
			ptrcloud_original->points[i].y = color_original->points[i].y;
			ptrcloud_original->points[i].z = color_original->points[i].z;
			int row = color_original->height - 1 - floor(i / color_original->width);
			//int row = floor(i / width);
			int col = i % color_original->width;
			uchar* ptr = image.ptr<uchar>(row);
			for (int j = col * image.channels(); j < (col + 1)*image.channels(); j++)
			{
				ptr[j] = color_original->points[i].r;
			}
		}
		ptrcloud_original->height = color_original->height;
		ptrcloud_original->width = color_original->width;
		ptrcloud_original->is_dense = color_original->is_dense;
		std::cout << "original points size is:" << ptrcloud_original->points.size() << std::endl;


		cvtColor(image, image, cv::COLOR_BGR2GRAY);
		float thresholdValue = OtsuAlgThreshold(image);

		pcl::PointIndices::Ptr inliers_visual_I(new pcl::PointIndices());
		for (int i = 0; i < color_original->size(); i++)
		{
			if (color_original->points[i].r > thresholdValue)
			{
				color_original->points[i].r = 255;
				color_original->points[i].g = 0;
				color_original->points[i].b = 0;
				inliers_visual_I->indices.push_back(i);
			}
		}

		pcl::ExtractIndices<pcl::PointXYZRGB> extract_visual_I;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual_I(new pcl::PointCloud<pcl::PointXYZRGB>);
		extract_visual_I.setInputCloud(color_original);
		extract_visual_I.setIndices(inliers_visual_I);
		extract_visual_I.setNegative(false);
		extract_visual_I.filter(*cloud_visual_I);

		//***********体素滤波*********/
		//pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
		//voxelGrid.setInputCloud(color_original);
		//voxelGrid.setLeafSize(2.5f, 2.5f, 2.5f); 

		////pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrFilteredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
		//voxelGrid.filter(*ptrFilteredCloud1);
		//std::cout << "VoxelGrid FilteredCloud1 points size is:" << ptrFilteredCloud1->points.size() << std::endl;

		///**********半径滤波*********/
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		//pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;	
		//ror.setInputCloud(ptrFilteredCloud1);							
		//ror.setRadiusSearch(4);			//ceil(6*sqrt(2))					
		//ror.setMinNeighborsInRadius(5);
		//ror.setNegative(false);
		//ror.filter(*ptrFilteredCloud);						
		//std::cout << "Radius FilteredCloud points size is:" << ptrFilteredCloud->points.size() << std::endl;

		pcl::PointXYZRGB minI;
		pcl::PointXYZRGB maxI;
		pcl::getMinMax3D(*cloud_visual_I, minI, maxI);
		float range = 20;
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud0(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::CropBox<pcl::PointXYZ> box_filter0;
		box_filter0.setMin(Eigen::Vector4f(minI.x - range, minI.y - range, minI.z - 20, 1.0));
		box_filter0.setMax(Eigen::Vector4f(maxI.x + range, maxI.y + range, maxI.z + 20, 1.0));
		box_filter0.setInputCloud(ptrcloud_original);
		box_filter0.setNegative(false);
		box_filter0.filter(*transformed_cloud0);

		/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::applyMorphologicalOperator<pcl::PointXYZ>(ptrFilteredCloud1, 5.0f, 2, *cloud_out);*/

		//PCA

		//don
		//体素最小矩形分割


		/*根据法线精分割*/
		pcl::PointXYZ min2;
		pcl::PointXYZ max2;
		min2.x = minI.x - range;
		min2.y = minI.y - range;
		min2.z = minI.z - range;
		max2.x = maxI.x + range;
		max2.y = maxI.y + range;
		max2.z = maxI.z + range;

		float sliceInterval_x = 10;
		float th = 0.5;//1.5;
		float edge_left = exactSeg_xleft(transformed_cloud0, min2, max2, sliceInterval_x, th);
		float edge_right = exactSeg_xright(transformed_cloud0, min2, max2, sliceInterval_x, th);

		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrFilteredx(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> passx;
		passx.setInputCloud(transformed_cloud0);
		passx.setFilterFieldName("x");
		passx.setFilterLimits(edge_left, edge_right);
		passx.filter(*ptrFilteredx);

		float edge_yleft = exactSeg_yleft(ptrFilteredx, min2, max2, sliceInterval_x, th);
		float edge_yright = exactSeg_yright(ptrFilteredx, min2, max2, sliceInterval_x, th);


		float length = edge_right - edge_left;
		float lengthaccuracy = 1 - abs((edge_right - edge_left - 198.10) / 198.10);
		float wigth = edge_yright - edge_yleft;
		float wigthaccuracy = 1 - abs((edge_yright - edge_yleft - 168.34) / 168.34);
		auto t2 = chrono::steady_clock::now();
		auto dt2 = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
		//fout <<  ti << " " << length << " " << lengthaccuracy << " " << wigth << " " << wigthaccuracy << " " << dt6<<endl;
		//cout << ti << endl;

		pcl::PointIndices::Ptr inliers_visual(new pcl::PointIndices());
		pcl::ExtractIndices<pcl::PointXYZ> extract_visual;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t i = 0; i < transformed_cloud0->size(); i++)
		{

			if (transformed_cloud0->points[i].x < edge_left || transformed_cloud0->points[i].x > edge_right
				|| transformed_cloud0->points[i].y < edge_yleft || transformed_cloud0->points[i].y > edge_yright)
			{
				/*cloud_visual->points[i].r = 255;
				cloud_visual->points[i].g = 0;
				cloud_visual->points[i].b = 0;*/
				inliers_visual->indices.push_back(i);

			}
			/*else
			{
				cloud_visual->points[i].r = 130;
				cloud_visual->points[i].g = 130;
				cloud_visual->points[i].b = 130;
			}*/

		}

		extract_visual.setInputCloud(transformed_cloud0);
		extract_visual.setIndices(inliers_visual);
		extract_visual.setNegative(true);
		extract_visual.filter(*cloud_visual);

		//pcl::visualization::PCLVisualizer my_viewer22;
		//my_viewer22.setBackgroundColor(255, 255, 255);
		//my_viewer22.addPointCloud<pcl::PointXYZRGB>(color_original);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color6(cloud_visual, 255, 0, 0);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_visual_I, 0, 255, 0);
		//my_viewer22.addPointCloud<pcl::PointXYZ>(cloud_visual, single_color6, "cloud_visual");
		//my_viewer22.addPointCloud<pcl::PointXYZRGB>(cloud_visual_I, single_color, "ptrcloud_original");
		////my_viewer2.addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(normals_pointsyright, normals_yright, 1, 2.0, "normals");
		//while (!my_viewer22.wasStopped())
		//{
		//	my_viewer22.spinOnce(100);
		//}

		pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_point(new pcl::PointCloud<pcl::PointXYZ>);
		boundary_point = boundary(cloud_visual);
		//直线度
		straightness(boundary_point);
		//平面度
		flatness(sample_count, transformed_cloud0, u_v);

		cout << ti << endl;
		ti++;
	}
	fout.close();
	system("pause");

	return (0);

}
