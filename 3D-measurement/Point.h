#ifndef __POINT_H__
#define __POINT_H__

#include <chrono>
// Include files for the PCL
// Suppress warnings coming from PCL headers.
#pragma warning(push, 1)
#pragma warning(disable : 4068) // unknown pragma
#pragma warning(disable : 4702) // unreachable code
#pragma push_macro("BOOST_ALLOW_DEPRECATED_HEADERS")
#ifndef BOOST_ALLOW_DEPRECATED_HEADERS
#    define BOOST_ALLOW_DEPRECATED_HEADERS
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#pragma GCC diagnostic pop
#pragma pop_macro("BOOST_ALLOW_DEPRECATED_HEADERS")
#pragma warning(pop)

// Include files to use the pylon API
#include <pylon/PylonIncludes.h>
#include <pylon/BlazeInstantCamera.h>

// Namespaces for using the pylon API and the blaze camera parameters
using namespace Pylon;
using namespace BlazeCameraParams_Params;
using namespace pcl;
using namespace std;

// Typedefs for the PCL types used
typedef PointXYZRGB Point_t;
typedef PointCloud<Point_t> PointCloud_t;
typedef PointCloud_t::Ptr PointCloudPtr;

typedef PointXYZ Point_xyz;
typedef PointCloud<Point_xyz> PointCloud_xyz;
typedef PointCloud_xyz::Ptr PointCloudPtrxyz;

class Sample
{
public:
	int run();
	bool onImageGrabbed(const CPylonDataContainer& container);

private:
	PointCloudPtr convertGrabResultToPointCloud(const CPylonDataContainer& container);
	//void loadPointClouds();
	int m_minDepth;                        // Minimum distance value [mm]
	int m_maxDepth;                        // Maximum distance value [mm]
	const float m_invalidDataValue = 0.0f; //  Value that identifies an invalid pixel

private:
	CBlazeInstantCamera m_camera;
	//uint32_t m_nPointCloudsSaved = 0;
	static const uint32_t c_countOfPointCloudsToSave = 20;
};

//#pragma pack(push, 1)
//struct Point
//{
//	float x;
//	float y;
//	float z;
//};
//#pragma pack(pop)
//
//class ToPointCloud
//{
//public:
//	int run();
//	//int run1();
//	bool onImageGrabbed(const CPylonDataContainer& container);
//
//private:
//	PointCloudPtr convertGrabResultToPointCloud(const CPylonDataContainer& container);
//	//PointCloud<PointXYZRGB>::Ptr convertGrabResultToPointCloud(const CPylonDataContainer& container);
//	//void loadAndShowPointClouds();
//	PointCloudPtr loadAndShowPointClouds();
//	//PointCloud<PointXYZRGB>::Ptr loadAndShowPointClouds();
//
//private:
//	CBlazeInstantCamera m_camera;
//	uint32_t m_nPointCloudsSaved = 0;
//	static const uint32_t c_countOfPointCloudsToSave = 1;
//};
#endif