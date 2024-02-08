#include <pcl/kdtree/kdtree.h>
#include <pcl/features/boundary.h>
#include<pcl/features/normal_3d.h>
#include "Point.h"


PointCloudPtrxyz boundary(PointCloudPtrxyz cloud_visual)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_visual);
	//tree2->setInputCloud(ptrFilteredCloud1);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	//normal_estimator.setSearchMethod(tree2);
	normal_estimator.setInputCloud(cloud_visual);
	//normal_estimator.setInputCloud(ptrFilteredCloud1);
	normal_estimator.setKSearch(6);//6
	normal_estimator.compute(*normals);
	//auto t4 = chrono::steady_clock::now();
	//auto dt4 = chrono::duration_cast<chrono::duration<double>>(t4 - t1).count();
	//cout << "time cost of normal estimation: " << dt4 << endl;

	pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
	boundaries->resize(cloud_visual->size()); //初始化大小
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
	boundary_estimation.setInputCloud(cloud_visual); //设置输入点云
	boundary_estimation.setInputNormals(normals); //设置输入法线
	boundary_estimation.setSearchMethod(tree2); //设置搜寻k近邻的方式
	boundary_estimation.setKSearch(15); //设置k近邻数量
	boundary_estimation.setAngleThreshold(M_PI / 2); //设置角度阈值，大于阈值为边界
	boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中
	/*auto t5 = chrono::steady_clock::now();
	auto dt5 = chrono::duration_cast<chrono::duration<double>>(t5 - t4).count();*/
	//cout << "time cost of boundary estimation: " << dt5 << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_point(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < cloud_visual->size(); i++)
	{
		if (boundaries->points[i].boundary_point != 0)
			boundary_point->push_back(cloud_visual->points[i]);
	}
	return boundary_point;
}




////	/*切线方向区域增长找边界*/
//	pcl::search::Search<pcl::PointXYZ>::Ptr tree_boundary(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree_boundary->setInputCloud(cloud_visual);
//	int K = 30;
//	std::vector<int> pointIdxKNNSearch(K);
//	std::vector<float> pointKNNSquaredDistance(K);
//	
//	pcl::PointCloud <pcl::Normal>::Ptr normals_boundary(new pcl::PointCloud <pcl::Normal>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr normalboundary(new pcl::PointCloud<pcl::PointXYZ>);
//	//pcl::PointCloud <pcl::Normal>::Ptr normals_boundary(new pcl::PointCloud <pcl::Normal>);
//	normals_boundary->resize(boundary_point->size());
//	normalboundary->resize(boundary_point->size());
//	for (int i = 0; i < boundary_point->size(); i++)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr Neighbors_boundary(new pcl::PointCloud<pcl::PointXYZ>);
//		if (tree_boundary->nearestKSearch(boundary_point->points[i], K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
//		{
//			Neighbors_boundary->push_back(boundary_point->points[i]);
//			for (int j = 0; j < pointIdxKNNSearch.size()-2; ++j)
//			{
//				Neighbors_boundary->push_back(boundary_point->points[pointIdxKNNSearch[j+1]]);
//			}
//		
//			pcl::ModelCoefficients::Ptr coefficients_boundary(new pcl::ModelCoefficients);
//			pcl::PointIndices::Ptr inliers_boundary(new pcl::PointIndices);
//			pcl::SACSegmentation<pcl::PointXYZ> seg_boundary;
//			seg_boundary.setOptimizeCoefficients(true);
//			seg_boundary.setModelType(pcl::SACMODEL_LINE);
//			seg_boundary.setDistanceThreshold(0.3);
//			seg_boundary.setInputCloud(Neighbors_boundary);
//			seg_boundary.segment(*inliers_boundary, *coefficients_boundary);
//
//			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual1(new pcl::PointCloud<pcl::PointXYZRGB>);
//			//cloud_visual1->resize(Neighbors_boundary->size());
//			//for (int i = 0; i < Neighbors_boundary->size(); i++)
//			//{
//			//	cloud_visual1->points[i].x = Neighbors_boundary->points[i].x;
//			//	cloud_visual1->points[i].y = Neighbors_boundary->points[i].y;
//			//	cloud_visual1->points[i].z = Neighbors_boundary->points[i].z;
//
//			//	cloud_visual1->points[i].r = 130;
//			//	cloud_visual1->points[i].g = 130;
//			//	cloud_visual1->points[i].b = 130;
//			//}
//			//for (int i = 0; i < inliers_boundary->indices.size(); i++)
//			//{
//			//	cloud_visual1->points[inliers_boundary->indices[i]].r = 255;
//			//	cloud_visual1->points[inliers_boundary->indices[i]].g = 0;
//			//	cloud_visual1->points[inliers_boundary->indices[i]].b = 0;
//			//}
//			///*pcl::ExtractIndices<pcl::PointXYZ> my_extract_indices;
//			//my_extract_indices.setInputCloud(cloud_belt);
//			//my_extract_indices.setIndices(inliers);
//			//my_extract_indices.setNegative(false);
//			//my_extract_indices.filter(*ptrcloud_result);*/
//			//pcl::visualization::PCLVisualizer my_viewer1;
//			//my_viewer1.setBackgroundColor(0,0,0);
//			//my_viewer1.addPointCloud<pcl::PointXYZRGB>(cloud_visual1);
//			//while (!my_viewer1.wasStopped())
//			//{
//			//	my_viewer1.spinOnce(100); // Spin until 'Q' is pressed
//			//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//			//}
//
//			//normalboundary->push_back(boundary_point->points[i]);
//			normalboundary->points[i].x = boundary_point->points[i].x;
//			normalboundary->points[i].y = boundary_point->points[i].y;
//			normalboundary->points[i].z = boundary_point->points[i].z;
//			/*normals_boundary->points[i].x = boundary_point->points[i].x;
//			normals_boundary->points[i].y = boundary_point->points[i].y;
//			normals_boundary->points[i].z = boundary_point->points[i].z;*/
//			normals_boundary->points[i].normal_x = coefficients_boundary->values[3];
//			normals_boundary->points[i].normal_y = coefficients_boundary->values[4];
//			normals_boundary->points[i].normal_z = coefficients_boundary->values[5];
//		}
//	}
//
//	pcl::visualization::PCLVisualizer my_viewer2;
//	my_viewer2.setBackgroundColor(0, 0, 0);
//	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(ptrcloud_original, 255, 0, 0);
//	my_viewer2.addPointCloud<pcl::PointXYZ>(normalboundary, "normalboundary");
//	my_viewer2.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(normalboundary, normals_boundary, 1, 1.0, "normals");
//	while (!my_viewer2.wasStopped())
//	{
//		my_viewer2.spinOnce(100);
//		//	//my_viewer2.spinOnce(100);
//	}
//////
//	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
//	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
//	pc.setInputCloud(boundary_point);
//	pc.setInputNormals(normals_boundary);
//	pc.setSearchMethod(tree_boundary);
//	pc.setKSearch(K);
//	//pc.setRadiusSearch(0.02);
//	//pc.setKSearch(5);
//	pc.compute(*cloud_curvatures);
//
//	float k1 = 0;
//	float k2 = 0;
//	for (int i = 0; i < cloud_curvatures->size(); i++)
//	{
//		k1 = cloud_curvatures->points[i].pc1;
//		k2 = cloud_curvatures->points[i].pc2;
//		normals_boundary->points[i].curvature = 0.5*(k1 + k2);
//	}
//	//cout << k1 << endl;
//	pcl::IndicesPtr indices_region(new std::vector <int>);
//	pcl::removeNaNFromPointCloud(*boundary_point, *indices_region);
//	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg_region;
//	reg_region.setMinClusterSize(20);
//	reg_region.setMaxClusterSize(10000);
//	reg_region.setSearchMethod(tree_boundary);
//	reg_region.setNumberOfNeighbours(20);//8
//	reg_region.setInputCloud(boundary_point);
//	reg_region.setIndices(indices_region);
//	reg_region.setInputNormals(normals_boundary);
//	reg_region.setSmoothnessThreshold(8/ 180.0 *M_PI);
//	reg_region.setCurvatureThreshold(0.05);
//
//	std::vector <pcl::PointIndices> clusters_boundary;
//	reg_region.extract(clusters_boundary);
//
//	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> RegionGrow_boundary; //储存区域增长分割后的点云
//	for (std::vector<pcl::PointIndices>::const_iterator it = clusters_boundary.begin(); it != clusters_boundary.end(); ++it)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
//			cloud_cluster->points.push_back(boundary_point->points[*pit]);
//		cloud_cluster->width = boundary_point->points.size();
//		cloud_cluster->height = 1;
//		cloud_cluster->is_dense = true;
//		RegionGrow_boundary.push_back(cloud_cluster);
//	}

//	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
//	viewer.setBackgroundColor(255, 255, 255);
//	for (int i = 0; i < RegionGrow_boundary.size(); i++)
//	{
//		////显示分割得到的各片点云
//		std::ostringstream str_filename;
//		str_filename << i << ".pcd";
//		//显示分割得到的各片点云 
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(RegionGrow_boundary[i], 255 * (1 - i)*(2 - i) / 2, 255 * i*(2 - i), 255 * i*(i - 1) / 2);
//		viewer.addPointCloud(RegionGrow_boundary[i], color, str_filename.str());
//		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, str_filename.str());
//	}
//
//
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce(100);
//		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
