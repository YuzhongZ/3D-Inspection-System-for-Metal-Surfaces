
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>

#include "Point.h"

float exactSeg_xleft(PointCloudPtrxyz transformed_cloud0, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th)
{

	int sliceNum_x = floor((max2.x - min2.x) / sliceInterval_x);
	float begin_xleft2 = min2.x;
	std::vector<float> angle_xleft2;
	float angle = 0;
	float edge_left = 0;
	pcl::PointCloud <pcl::PointNormal>::Ptr normals_xleft(new pcl::PointCloud <pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normals_pointsleft(new pcl::PointCloud<pcl::PointXYZ>);
	/*pcl::PointCloud <pcl::PointNormal>::Ptr normals_x(new pcl::PointCloud <pcl::PointNormal>);
	normals_x->resize(4 * sliceNum_x);*/
	int num_x = 0;
	normals_xleft->resize(sliceNum_x);
	normals_pointsleft->resize(sliceNum_x);
	float base_xleft = begin_xleft2;
	for (int num = 0; num < sliceNum_x; num++)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(transformed_cloud0);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(base_xleft, sliceInterval_x + base_xleft);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_belt(new pcl::PointCloud<pcl::PointXYZ>);
		pass.filter(*cloud_belt);
		float sum_z = 0;
		for (int i = 0; i < cloud_belt->points.size(); i++)
		{
			sum_z = sum_z + cloud_belt->points[i].z;
		}
		float averz = sum_z / cloud_belt->points.size();
		/*normals_x->points[num_x].x = base_xleft;
		normals_x->points[num_x].y = 0;
		normals_x->points[num_x].z = averz;*/
		normals_xleft->points[num].x = base_xleft;
		normals_xleft->points[num].y = 0;
		normals_xleft->points[num].z = averz;
		normals_pointsleft->points[num].x = base_xleft;
		normals_pointsleft->points[num].y = 0;
		normals_pointsleft->points[num].z = averz;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(50);
		seg.setDistanceThreshold(th);
		seg.setInputCloud(cloud_belt);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < 3)
		{
			std::cerr << "outlier" << std::endl;
			normals_xleft->points[num].normal_x = std::numeric_limits<float>::quiet_NaN();
			normals_xleft->points[num].normal_y = std::numeric_limits<float>::quiet_NaN();
			normals_xleft->points[num].normal_z = std::numeric_limits<float>::quiet_NaN();
			normals_xleft->points[num].curvature = std::numeric_limits<float>::quiet_NaN();
			normals_xleft->is_dense = false;
			continue;
		}
		else
		{
			/*normals_x->points[num_x].normal_x = coefficients->values[0];
			normals_x->points[num_x].normal_y = coefficients->values[1];
			normals_x->points[num_x].normal_z = coefficients->values[2];*/
			normals_xleft->points[num].normal_x = coefficients->values[0];
			normals_xleft->points[num].normal_y = coefficients->values[1];
			normals_xleft->points[num].normal_z = coefficients->values[2];
			flipNormalTowardsViewpoint(normals_xleft->points[num], 0, 0, 500, normals_xleft->points[num].normal_x, normals_xleft->points[num].normal_y, normals_xleft->points[num].normal_z);
			//normals->points[num].curvature = std::numeric_limits<float>::quiet_NaN();

			Eigen::Vector3d gi0(0, 0, 1);
			Eigen::Vector3d gii(normals_xleft->points[num].normal_x, normals_xleft->points[num].normal_y, normals_xleft->points[num].normal_z);
			angle = 180 * acos(gi0.dot(gii.normalized())) / M_PI;
			angle_xleft2.push_back(angle);
			if (num != 0 && angle > 35 && angle < 90 && normals_xleft->points[num].normal_x > 0)
			{
				edge_left = normals_xleft->points[num].x + 0.5 *sliceInterval_x;
				break;
			}

		}
		base_xleft = base_xleft + 0.5 *sliceInterval_x;
		//num_x++;
	}
	return edge_left;
}

float exactSeg_xright(PointCloudPtrxyz transformed_cloud0, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th)
{
	int sliceNum_x = floor((max2.x - min2.x) / sliceInterval_x);
	float begin_xright2 = max2.x;
	std::vector<float> angle_xright2;
	float angle = 0;
	float edge_right = 0;
	pcl::PointCloud <pcl::PointNormal>::Ptr normals_xright(new pcl::PointCloud <pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normals_pointsright(new pcl::PointCloud<pcl::PointXYZ>);
	normals_xright->resize(sliceNum_x);
	normals_pointsright->resize(sliceNum_x);
	float base_xright = begin_xright2;
	for (int num = 0; num < sliceNum_x; num++)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(transformed_cloud0);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(base_xright - sliceInterval_x, base_xright);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_belt(new pcl::PointCloud<pcl::PointXYZ>);
		pass.filter(*cloud_belt);
		float sum_z = 0;
		for (int i = 0; i < cloud_belt->points.size(); i++)
		{
			sum_z = sum_z + cloud_belt->points[i].z;
		}
		float averz = sum_z / cloud_belt->points.size();
		/*normals_x->points[num_x].x = base_xright;
		normals_x->points[num_x].y = 0;
		normals_x->points[num_x].z = averz;*/
		normals_xright->points[num].x = base_xright;
		normals_xright->points[num].y = 0;
		normals_xright->points[num].z = averz;
		normals_pointsright->points[num].x = base_xright;
		normals_pointsright->points[num].y = 0;
		normals_pointsright->points[num].z = averz;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(50);
		seg.setDistanceThreshold(th);
		seg.setInputCloud(cloud_belt);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < 3)
		{
			std::cerr << "outlier" << std::endl;
			normals_xright->points[num].normal_x = std::numeric_limits<float>::quiet_NaN();
			normals_xright->points[num].normal_y = std::numeric_limits<float>::quiet_NaN();
			normals_xright->points[num].normal_z = std::numeric_limits<float>::quiet_NaN();
			normals_xright->points[num].curvature = std::numeric_limits<float>::quiet_NaN();
			normals_xright->is_dense = false;
			continue;
		}
		else
		{
			/*normals_x->points[num_x].normal_x = coefficients->values[0];
			normals_x->points[num_x].normal_y = coefficients->values[1];
			normals_x->points[num_x].normal_z = coefficients->values[2];*/
			normals_xright->points[num].normal_x = coefficients->values[0];
			normals_xright->points[num].normal_y = coefficients->values[1];
			normals_xright->points[num].normal_z = coefficients->values[2];
			flipNormalTowardsViewpoint(normals_xright->points[num], 0, 0, 500, normals_xright->points[num].normal_x, normals_xright->points[num].normal_y, normals_xright->points[num].normal_z);
			//normals->points[num].curvature = std::numeric_limits<float>::quiet_NaN();

			Eigen::Vector3d gi0(0, 0, 1);
			Eigen::Vector3d gii(normals_xright->points[num].normal_x, normals_xright->points[num].normal_y, normals_xright->points[num].normal_z);
			angle = 180 * acos(gi0.dot(gii.normalized())) / M_PI;
			//gradient = gradient + angle;
			angle_xright2.push_back(angle);
			if (num != 0 && angle > 35 && angle < 90 && normals_xright->points[num].normal_x < 0)
			{
				edge_right = normals_xright->points[num].x - 0.5 *sliceInterval_x;//0.5*  sliceInterval_x;
				break;
			}

		}
		base_xright = base_xright - 0.5 *sliceInterval_x;//0.5 * sliceInterval_x;
		//num_x++;
	}
	return edge_right;
}



float exactSeg_yleft(PointCloudPtrxyz ptrFilteredx, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th)
{
	float sliceInterval_y = sliceInterval_x;
	int sliceNum_y = floor((max2.y - min2.y) / sliceInterval_y);
	float begin_yleft2 = min2.y;
	std::vector<float> angle_yleft2;
	float angle_y = 0;
	float edge_yleft = 0;
	pcl::PointCloud <pcl::PointNormal>::Ptr normals_yleft(new pcl::PointCloud <pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normals_pointsyleft(new pcl::PointCloud<pcl::PointXYZ>);
	normals_yleft->resize(sliceNum_y);
	normals_pointsyleft->resize(sliceNum_y);
	float base_yleft = begin_yleft2;
	for (int num = 0; num < sliceNum_y; num++)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(ptrFilteredx);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(base_yleft, sliceInterval_y + base_yleft);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_belt(new pcl::PointCloud<pcl::PointXYZ>);
		pass.filter(*cloud_belt);
		float sum_z = 0;
		for (int i = 0; i < cloud_belt->points.size(); i++)
		{
			sum_z = sum_z + cloud_belt->points[i].z;
		}
		float averz = sum_z / cloud_belt->points.size();
		normals_yleft->points[num].x = 0;
		normals_yleft->points[num].y = base_yleft;
		normals_yleft->points[num].z = averz;
		normals_pointsyleft->points[num].x = 0;
		normals_pointsyleft->points[num].y = base_yleft;
		normals_pointsyleft->points[num].z = averz;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(50);
		seg.setDistanceThreshold(th);
		seg.setInputCloud(cloud_belt);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < 3)
		{
			std::cerr << "outlier" << std::endl;
			normals_yleft->points[num].normal_x = std::numeric_limits<float>::quiet_NaN();
			normals_yleft->points[num].normal_y = std::numeric_limits<float>::quiet_NaN();
			normals_yleft->points[num].normal_z = std::numeric_limits<float>::quiet_NaN();
			normals_yleft->points[num].curvature = std::numeric_limits<float>::quiet_NaN();
			normals_yleft->is_dense = false;
			continue;
		}
		else
		{
			normals_yleft->points[num].normal_x = coefficients->values[0];
			normals_yleft->points[num].normal_y = coefficients->values[1];
			normals_yleft->points[num].normal_z = coefficients->values[2];
			flipNormalTowardsViewpoint(normals_yleft->points[num], 0, 0, 500, normals_yleft->points[num].normal_x, normals_yleft->points[num].normal_y, normals_yleft->points[num].normal_z);
			//normals->points[num].curvature = std::numeric_limits<float>::quiet_NaN();

			Eigen::Vector3d gi0(0, 0, 1);
			Eigen::Vector3d gii(normals_yleft->points[num].normal_x, normals_yleft->points[num].normal_y, normals_yleft->points[num].normal_z);
			angle_y = 180 * acos(gi0.dot(gii.normalized())) / M_PI;
			angle_yleft2.push_back(angle_y);
			if (num != 0 && angle_y > 35 && angle_y < 90 && normals_yleft->points[num].normal_y > 0)
			{
				edge_yleft = normals_yleft->points[num].y + 0.5 *sliceInterval_x;
				break;
			}

		}
		base_yleft = base_yleft + 0.5 *sliceInterval_x;

	}

	return edge_yleft;
}

float exactSeg_yright(PointCloudPtrxyz ptrFilteredx, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th)
{
	int sliceNum_y = floor((max2.y - min2.y) / sliceInterval_x);
	float begin_yright2 = max2.y;
	std::vector<float> angle_yright2;
	float angle_y = 0;
	float edge_yright = 0;
	pcl::PointCloud <pcl::PointNormal>::Ptr normals_yright(new pcl::PointCloud <pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normals_pointsyright(new pcl::PointCloud<pcl::PointXYZ>);
	normals_yright->resize(sliceNum_y);
	normals_pointsyright->resize(sliceNum_y);
	float base_yright = begin_yright2;
	for (int num = 0; num < sliceNum_y; num++)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(ptrFilteredx);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(base_yright - sliceInterval_x, base_yright);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_belt(new pcl::PointCloud<pcl::PointXYZ>);
		pass.filter(*cloud_belt);
		float sum_z = 0;
		for (int i = 0; i < cloud_belt->points.size(); i++)
		{
			sum_z = sum_z + cloud_belt->points[i].z;
		}
		float averz = sum_z / cloud_belt->points.size();
		normals_yright->points[num].x = 0;
		normals_yright->points[num].y = base_yright;
		normals_yright->points[num].z = averz;
		normals_pointsyright->points[num].x = 0;
		normals_pointsyright->points[num].y = base_yright;
		normals_pointsyright->points[num].z = averz;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(50);
		seg.setDistanceThreshold(th);
		seg.setInputCloud(cloud_belt);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < 3)
		{
			std::cerr << "outlier" << std::endl;
			normals_yright->points[num].normal_x = std::numeric_limits<float>::quiet_NaN();
			normals_yright->points[num].normal_y = std::numeric_limits<float>::quiet_NaN();
			normals_yright->points[num].normal_z = std::numeric_limits<float>::quiet_NaN();
			normals_yright->points[num].curvature = std::numeric_limits<float>::quiet_NaN();
			normals_yright->is_dense = false;
			continue;
		}
		else
		{
			normals_yright->points[num].normal_x = coefficients->values[0];
			normals_yright->points[num].normal_y = coefficients->values[1];
			normals_yright->points[num].normal_z = coefficients->values[2];
			flipNormalTowardsViewpoint(normals_yright->points[num], 0, 0, 500, normals_yright->points[num].normal_x, normals_yright->points[num].normal_y, normals_yright->points[num].normal_z);
			//normals->points[num].curvature = std::numeric_limits<float>::quiet_NaN();

			Eigen::Vector3d gi0(0, 0, 1);
			Eigen::Vector3d gii(normals_yright->points[num].normal_x, normals_yright->points[num].normal_y, normals_yright->points[num].normal_z);
			angle_y = 180 * acos(gi0.dot(gii.normalized())) / M_PI;
			angle_yright2.push_back(angle_y);
			if (num != 0 && angle_y > 35 && angle_y < 90 && normals_yright->points[num].normal_y < 0)
			{
				edge_yright = normals_yright->points[num].y - 0.5 *sliceInterval_x;
				break;
			}

		}
		base_yright = base_yright - 0.5 *sliceInterval_x;

	}
	return edge_yright;

}

