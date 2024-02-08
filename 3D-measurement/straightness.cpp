#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Point.h"

void straightness(PointCloudPtrxyz boundary_point)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr tryline1(new pcl::PointCloud<pcl::PointXYZ>);
	tryline1->resize(boundary_point->size());
	for (int i = 0; i < boundary_point->size(); i++)
	{
		tryline1->points[i] = boundary_point->points[i];
		tryline1->points[i].z = 0;
	}

	int count = 0;
	int max_count = 4;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remain(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_line(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> ModelLine; //注意如果里面是pcl::PointCloud<pcl::PointXYZ>::Ptr原来数据会被新的覆盖
	std::vector<pcl::ModelCoefficients> ModelLinecoefficients;
	pcl::PointXYZ mincopyboundary;
	pcl::PointXYZ maxcopyboundary;
	pcl::getMinMax3D(*boundary_point, mincopyboundary, maxcopyboundary);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrboundary(new pcl::PointCloud<pcl::PointXYZ>);
	while (count < max_count)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;             
		pass.setInputCloud(tryline1);//boundary_point
		switch (count)
		{
		case 0: {
			pass.setFilterFieldName("y");
			pass.setFilterLimits(mincopyboundary.y, mincopyboundary.y + 6);
			break;
		}
		case 1: {
			pass.setFilterFieldName("y");
			pass.setFilterLimits(maxcopyboundary.y - 6, maxcopyboundary.y);
			break;
		}
		case 2: {
			pass.setFilterFieldName("x");
			pass.setFilterLimits(mincopyboundary.x, mincopyboundary.x + 6);
			break;
		}
		case 3: {
			pass.setFilterFieldName("x");
			pass.setFilterLimits(maxcopyboundary.x - 6, maxcopyboundary.x);
			break;
		}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_belt(new pcl::PointCloud<pcl::PointXYZ>);
		pass.filter(*cloud_belt);

		pcl::ModelCoefficients::Ptr coefficients_boundary(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_boundary(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg_boundary;
		seg_boundary.setOptimizeCoefficients(true);
		seg_boundary.setModelType(pcl::SACMODEL_LINE);
		seg_boundary.setDistanceThreshold(0.8);//1.6
		seg_boundary.setInputCloud(cloud_belt);
		seg_boundary.segment(*inliers_boundary, *coefficients_boundary);

		pcl::copyPointCloud<pcl::PointXYZ>(*cloud_belt, *inliers_boundary, *model_line);
		ModelLine.push_back(*model_line);
		ModelLinecoefficients.push_back(*coefficients_boundary);

		count++;
	}

	///*用有z值的去拟合直线去掉偏离点再扩大*/
	/*pcl::search::Search<pcl::PointXYZ>::Ptr tree_yline(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_yline->setInputCloud(tryline1);
	int K1 = 1;
	std::vector<int> pointIdxKNNSearch1;
	std::vector<float> pointKNNSquaredDistance1;
	pcl::PointIndices::Ptr inliers_boundaryline1(new pcl::PointIndices());
	for (int k = 0; k < max_count; k++)
	{
		for (int i = 0; i < ModelLine[k].size(); i++)
		{
			if (tree_yline->nearestKSearch(ModelLine[k].points[i], K1, pointIdxKNNSearch1, pointKNNSquaredDistance1) > 0)
			{
				inliers_boundaryline1->indices.push_back(pointIdxKNNSearch1[0]);
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_remain(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract_boundary;
	extract_boundary.setInputCloud(boundary_point);
	extract_boundary.setIndices(inliers_boundaryline1);
	extract_boundary.setNegative(false);
	extract_boundary.filter(*boundary_remain);*/


	//pcl::search::Search<pcl::PointXYZ>::Ptr tree_yline(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree_yline->setInputCloud(boundary_point); 
	//float radius = 1.0;//1.2
	//pcl::PointIndices::Ptr inliers_boundaryline(new pcl::PointIndices());
	//std::vector<pcl::PointCloud<pcl::PointXYZ>> filteredLine;
	//for (int k = 0; k < max_count; k++)
	//{
	//	std::vector<int> pointIdxRadiusSearch;
	//	std::vector<float> pointRadiusSquaredDistance;
	//	pcl::PointIndices::Ptr inliers_liney(new pcl::PointIndices());
	//	for (int i = 0; i < ModelLine[k].size(); i++)
	//	{
	//		//pcl::PointCloud<pcl::PointXYZ>::Ptr Neighbors_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	//		if (tree_yline->radiusSearch(ModelLine[k].points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	//		{
	//			for (int j = 0; j < pointIdxRadiusSearch.size(); ++j)
	//			{
	//				if (find(inliers_boundaryline->indices.begin(), inliers_boundaryline->indices.end(), pointIdxRadiusSearch[j]) == inliers_boundaryline->indices.end())
	//				{
	//					inliers_boundaryline->indices.push_back(pointIdxRadiusSearch[j]);
	//					inliers_liney->indices.push_back(pointIdxRadiusSearch[j]);
	//				}
	//			}
	//		}
	//	}
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr liney_remain(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::copyPointCloud<pcl::PointXYZ>(*boundary_point, *inliers_liney, *liney_remain);
	//	filteredLine.push_back(*liney_remain);
	//}
	//////	pcl::visualization::PCLVisualizer viewer3("PCLVisualizer3");
	////   //  viewer3.addPointCloud(liney_remain);
	////   // 	while (!viewer3.wasStopped())
	//////	{
	//////		viewer3.spinOnce(100);
	//////	}
	////
	////
	//pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_remain(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ExtractIndices<pcl::PointXYZ> extract_boundary;
	//extract_boundary.setInputCloud(boundary_point);
	//extract_boundary.setIndices(inliers_boundaryline);
	//extract_boundary.setNegative(false);
	//extract_boundary.filter(*boundary_remain);

	////pcl::visualization::PCLVisualizer viewer3("PCLVisualizer3");
	//////viewer3.addPointCloud(boundary_remain);
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(boundary_point, 255, 255, 255);
	////viewer3.addPointCloud(boundary_point, color1, "cloud_belt");
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(boundary_remain, 255, 0, 0);
	////viewer3.addPointCloud(boundary_remain, color2, "boundary_remain");
	////while (!viewer3.wasStopped())
	////{
	////	viewer3.spinOnce(100);
	////}



	///*点到直线的距离*/
	float dist_line, dist_linemax, dist_linemin, a, b, c, straightness, sum_x1, sum_x2, sum_xy, sum_y1, Dls, uls, u1, u0, ut;
	std::vector<float> pointlinestraightness;
	Eigen::MatrixXd matA(2, 2);
	Eigen::VectorXd matB(2);
	Eigen::VectorXd coef;
	pcl::PointXYZ dismaxpointLS, disminpointLS;
	float u2 = 5 / 3;
	float u3 = 1 / 3;
	float T = 0.4;
	for (int k = 0; k < 2; k++)//
	{
		dist_linemax = -100;
		dist_linemin = 100;
		//std::vector<float> distpointline;
		sum_x1 = 0;
		sum_x2 = 0;
		sum_xy = 0;
		sum_y1 = 0;
		ut = 0;
		//u1 = (sqrt(2)*T) / sqrt(filteredLine[k].size());//( 6*sqrt(samplesize))
		u1 = (sqrt(2)*T) / sqrt(ModelLine[k].size());//( 6*sqrt(samplesize))
		u0 = sqrt(u1*u1 + u2 * u2 + u3 * u3);
		//Eigen::VectorXd matX(1,filteredLine[k].size());
		for (int i = 0; i < ModelLine[k].size(); i++)
		{
			//matX(0,i)= filteredLine[k].points[i].x;
			sum_x2 = sum_x2 + (ModelLine[k].points[i].x)*(ModelLine[k].points[i].x);
			sum_x1 = sum_x1 + ModelLine[k].points[i].x;
			sum_xy = sum_xy + ModelLine[k].points[i].x * ModelLine[k].points[i].y;
			sum_y1 = sum_y1 + ModelLine[k].points[i].y;

		}
		//for (int i = 0; i < filteredLine[k].size(); i++)
		//{
		//	//matX(0,i)= filteredLine[k].points[i].x;
		//	sum_x2 = sum_x2 + (filteredLine[k].points[i].x)*(filteredLine[k].points[i].x);
		//	sum_x1 = sum_x1 + filteredLine[k].points[i].x;
		//	sum_xy = sum_xy + filteredLine[k].points[i].x * filteredLine[k].points[i].y;
		//	sum_y1 = sum_y1 + filteredLine[k].points[i].y;

		//}
		//Dls = (filteredLine[k].size() + 1)*sum_x2 - sum_x1 * sum_x1;
		Dls = (ModelLine[k].size() + 1)*sum_x2 - sum_x1 * sum_x1;
		matA(0, 0) = sum_x2;
		matA(0, 1) = sum_x1;
		matA(1, 0) = sum_x1;
		//matA(1, 1) = filteredLine[k].size();
		matA(1, 1) = ModelLine[k].size();
		matB(0) = sum_xy;
		matB(1) = sum_y1;
		coef = matA.inverse() *matB;
		//cout << coef << endl;
		a = coef[0];
		b = -1;
		c = coef[1];

		//for (int i = 0; i < filteredLine[k].size(); i++)
		//{
		//	ut += ((filteredLine[k].size() + 1)*filteredLine[k].points[i].x - sum_x1)*((filteredLine[k].size() + 1)*filteredLine[k].points[i].x - sum_x1);
		//	//dist_line = (a* filteredLine[k].points[i].x + b * filteredLine[k].points[i].y + c) / sqrt(a * a + b * b);
		//	dist_line = filteredLine[k].points[i].y - coef[0] * filteredLine[k].points[i].x - coef[1];
		//	//distpointline.push_back(dist_line);
		//	if (dist_line > dist_linemax)
		//	{
		//		dist_linemax = dist_line;
		//		dismaxpointLS = filteredLine[k].points[i];
		//	}
		//	else if (dist_line < dist_linemin)
		//	{
		//		dist_linemin = dist_line;
		//		disminpointLS = filteredLine[k].points[i];
		//	}
		//}
		//ut = ut - ((filteredLine[k].size() + 1)*dismaxpointLS.x - sum_x1)*((filteredLine[k].size() + 1)*dismaxpointLS.x - sum_x1) - ((filteredLine[k].size() + 1)*disminpointLS.x - sum_x1)*((filteredLine[k].size() + 1)*disminpointLS.x - sum_x1);
		for (int i = 0; i < ModelLine[k].size(); i++)
		{
			ut += ((ModelLine[k].size() + 1)*ModelLine[k].points[i].x - sum_x1)*((ModelLine[k].size() + 1)*ModelLine[k].points[i].x - sum_x1);
			//dist_line = (a* filteredLine[k].points[i].x + b * filteredLine[k].points[i].y + c) / sqrt(a * a + b * b);
			dist_line = ModelLine[k].points[i].y - coef[0] * ModelLine[k].points[i].x - coef[1];
			//distpointline.push_back(dist_line);
			if (dist_line > dist_linemax)
			{
				dist_linemax = dist_line;
				dismaxpointLS = ModelLine[k].points[i];
			}
			else if (dist_line < dist_linemin)
			{
				dist_linemin = dist_line;
				disminpointLS = ModelLine[k].points[i];
			}
		}
		ut = ut - ((ModelLine[k].size() + 1)*dismaxpointLS.x - sum_x1)*((ModelLine[k].size() + 1)*dismaxpointLS.x - sum_x1) - ((ModelLine[k].size() + 1)*disminpointLS.x - sum_x1)*((ModelLine[k].size() + 1)*disminpointLS.x - sum_x1);
		straightness = dist_linemax - dist_linemin;
		pointlinestraightness.push_back(straightness);
		//cout << "Straightness: " << straightness << endl;
		//uls = sqrt(2 + ((matX.transpose()*matX).inverse()*(matX.transpose()*matX).inverse())*(dismaxpointLS.x - disminpointLS.x)*(dismaxpointLS.x - disminpointLS.x))*u0;
		/*uls = sqrt((Dls - (dismaxpointLS.x - disminpointLS.x)*((filteredLine[k].size() + 1)*dismaxpointLS.x - sum_x1))*(Dls - (dismaxpointLS.x - disminpointLS.x)*((filteredLine[k].size() + 1)*dismaxpointLS.x - sum_x1)) / (Dls*Dls)
			+ (Dls - (dismaxpointLS.x - disminpointLS.x)*((filteredLine[k].size() + 1)*disminpointLS.x - sum_x1))*(Dls - (dismaxpointLS.x - disminpointLS.x)*((filteredLine[k].size() + 1)*disminpointLS.x - sum_x1)) / (Dls*Dls)
			+ (dismaxpointLS.x - disminpointLS.x)*(dismaxpointLS.x - disminpointLS.x)*ut / (Dls*Dls))*u0;*/
		uls = sqrt((Dls - (dismaxpointLS.x - disminpointLS.x)*((ModelLine[k].size() + 1)*dismaxpointLS.x - sum_x1))*(Dls - (dismaxpointLS.x - disminpointLS.x)*((ModelLine[k].size() + 1)*dismaxpointLS.x - sum_x1)) / (Dls*Dls)
			+ (Dls - (dismaxpointLS.x - disminpointLS.x)*((ModelLine[k].size() + 1)*disminpointLS.x - sum_x1))*(Dls - (dismaxpointLS.x - disminpointLS.x)*((ModelLine[k].size() + 1)*disminpointLS.x - sum_x1)) / (Dls*Dls)
			+ (dismaxpointLS.x - disminpointLS.x)*(dismaxpointLS.x - disminpointLS.x)*ut / (Dls*Dls))*u0;
			//fout << " " << straightness << " " << uls ;
		cout << " straightness: " << straightness << "   uls:" << uls << endl;
	}
}
