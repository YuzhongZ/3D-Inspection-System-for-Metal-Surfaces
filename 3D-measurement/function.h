#pragma once

#include <opencv2/core/core.hpp> 
#include "Point.h"


int OtsuAlgThreshold(const cv::Mat image);
void flatness(int sample_count, PointCloudPtrxyz transformed_cloud0, std::vector<Eigen::Vector2f> u_v);
void straightness(PointCloudPtrxyz boundary_point);
PointCloudPtrxyz boundary(PointCloudPtrxyz cloud_visual);
float exactSeg_xleft(PointCloudPtrxyz transformed_cloud0, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th);
float exactSeg_xright(PointCloudPtrxyz transformed_cloud0, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th);
float exactSeg_yleft(PointCloudPtrxyz ptrFilteredx, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th);
float exactSeg_yright(PointCloudPtrxyz ptrFilteredx, Point_xyz min2, Point_xyz max2, float sliceInterval_x, float th);
