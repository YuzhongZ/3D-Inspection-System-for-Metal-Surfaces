#include "Point.h"
#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <csignal>
#include <cmath>
#include <mutex>
#include <atomic>
#include <functional>
#include <iostream>
using namespace pcl;
using namespace Pylon;
using namespace BlazeCameraParams_Params;
using namespace std;
using namespace cv;

#pragma pack(push, 1)
struct Pointd
{
    float x;
    float y;
    float z;
};
#pragma pack(pop)
#define USE_RADIAL_DISTANCES 0
uint32_t m_nPointCloudsSaved = 0;
//#define USE_OPENCV 1m_nPointCloudsSaved

// Checks whether a given 3D point represents a valid coordinate.
// The Scan3dInvalidDataValue is used to identify a non-valid pixel.
bool isValid(const Pointd* point, float invalidDataValue)
{
	return std::isnan(invalidDataValue) ? !std::isnan(point->z) : point->z != invalidDataValue;
}

// Calculates a grayscale depth map from point cloud data sent from a Basler blaze camera.
// The buffer that pDepthMap points to must be allocated accordingly before
// passing it to the calculateDepthMap function.
void calculateDepthMap(const CPylonDataComponent& rangeComponent,
	int minDepth,
	int maxDepth,
	void* pDepthMap,
	float invalidDataValue,
	bool useRadialDistances)
{
	const int width = rangeComponent.GetWidth();
	const int height = rangeComponent.GetHeight();
	auto pPoint = reinterpret_cast<const Pointd*>(rangeComponent.GetData());
	auto pDest = (uint16_t*)pDepthMap;

	const double scale = 65535.0 / (maxDepth - minDepth);

	for (int row = 0; row < height; ++row)
	{
		for (int col = 0; col < width; ++col, ++pPoint, ++pDest)
		{
			if (isValid(pPoint, invalidDataValue))
			{
				//double t1 = sqrt(pPoint->x * pPoint->x + pPoint->y * pPoint->y + pPoint->z * pPoint->z);
				//double t2= pPoint->z;
				double distance = useRadialDistances ?
					sqrt(pPoint->x * pPoint->x + pPoint->y * pPoint->y + pPoint->z * pPoint->z) :
					pPoint->z;
				// Clip to [minDepth..MaxDepth].
				if (distance < minDepth)
					distance = minDepth;
				else if (distance > maxDepth)
					distance = maxDepth;
				*pDest = (uint16_t)((distance - minDepth) * scale);
			}
			else
			{
				// No depth information available for this pixel. Zero it.
				*pDest = 0;
			}
		}
	}
}

#pragma pack(push, 1)
struct BGR
{
	uint8_t b;
	uint8_t g;
	uint8_t r;
};
#pragma pack(pop)

// Calculates a color depth map from point cloud data sent from a Basler blaze camera.
// The buffer that pDepthMap points to must be allocated accordingaly before
// passing it to the calculateDepthMap function.
void calculateDepthMapColor(const CPylonDataComponent& rangeComponent,
	int minDepth,
	int maxDepth,
	void* pDepthMap,
	float invalidDataValue,
	bool useRadialDistances)
{
	const int width = rangeComponent.GetWidth();
	const int height = rangeComponent.GetHeight();
	auto pPoint = reinterpret_cast<const Pointd*>(rangeComponent.GetData());
	auto pDest = (BGR*)pDepthMap;

	const double scale = 65535.0 / (maxDepth - minDepth);

	for (int row = 0; row < height; ++row)
	{
		for (int col = 0; col < width; ++col, ++pPoint, ++pDest)
		{
			if (isValid(pPoint, invalidDataValue))
			{
				double distance = useRadialDistances ?
					sqrt(pPoint->x * pPoint->x + pPoint->y * pPoint->y + pPoint->z * pPoint->z) :
					pPoint->z;
				// Clip to [minDepth..MaxDepth].
				if (distance < minDepth)
					distance = minDepth;
				else if (distance > maxDepth)
					distance = maxDepth;

				// Calculate the color.
				const uint16_t g = (uint16_t)((distance - minDepth) * scale);
				const uint16_t val = g >> 6 & 0xff;
				const uint16_t sel = g >> 14;
				uint32_t res = val << 8 | 0xff;
				if (sel & 0x01)
				{
					res = (~res) >> 8 & 0xffff;
				}
				if (sel & 0x02)
				{
					res = res << 8;
				}
				pDest->r = res & 0xff;
				res = res >> 8;
				pDest->g = res & 0xff;
				res = res >> 8;
				pDest->b = res & 0xff;
			}
			else
			{
				// No depth information available for this pixel. Set it to black.
				*pDest = {};
			}
		}
	}
}


/*
Convert the Basler point cloud to a PCL point cloud.
*/
PointCloudPtr Sample::convertGrabResultToPointCloud(const CPylonDataContainer& container)
{
	// An organized point cloud is used, i.e., for each camera pixel there is an entry
	// in the data structure indicating the 3D coordinates calculated from that pixel.

	if (container.GetDataComponentCount() == 0)
	{
		std::cerr << "No valid image data." << std::endl;
		return nullptr;
	}

	auto rangeComponent = container.GetDataComponent(0);
	auto intensityComponent = container.GetDataComponent(1);
	auto confidenceComponent = container.GetDataComponent(2);

	// Allocate PCL point cloud.
	const size_t width = rangeComponent.GetWidth();
	const size_t height = rangeComponent.GetHeight();
	PointCloudPtr ptrPointCloud(new PointCloud_t);
	ptrPointCloud->width = (uint32_t)width;
	ptrPointCloud->height = (uint32_t)height;
	ptrPointCloud->points.resize(width * height);
	ptrPointCloud->is_dense = false; // Indicates an organized point cloud.

	// Create a pointer to the 3D coordinates of the first point.
	// imgParts[0] always refers to the point cloud data.
	auto pSrcPoint = reinterpret_cast<const Pointd*>(rangeComponent.GetData());

	// Create a pointer to the intensity information, which is stored in the second buffer part.
	uint16_t* pIntensity = (uint16_t*)intensityComponent.GetData();

	// Set the points.
	for (size_t i = 0; i < height * width; ++i, ++pSrcPoint, ++pIntensity)
	{
		// Set the x/y/z coordinates.
		Point_t& dstPoint = ptrPointCloud->points[i];

		dstPoint.x = pSrcPoint->x;
		dstPoint.y = pSrcPoint->y;
		dstPoint.z = pSrcPoint->z;

		// Use the intensity value of the pixel for coloring the point.
		dstPoint.r = dstPoint.g = dstPoint.b = (uint8_t)(*pIntensity >> 8);
	}


	// Allocate OpenCV images for the depth maps and convert the point cloud data to
	// depth maps.
	
	cv::Mat depthImage((int)height, (int)width, CV_16UC1); // 2 byte per pixel
	calculateDepthMap(rangeComponent,
		m_minDepth,
		m_maxDepth,
		depthImage.ptr(),
		m_invalidDataValue,
		USE_RADIAL_DISTANCES);

	cv::Mat depthImageColor((int)height, (int)width, CV_8UC3); // 3 byte per pixel, BGR format.
	calculateDepthMapColor(rangeComponent,
		m_minDepth,
		m_maxDepth,
		depthImageColor.ptr(),
		m_invalidDataValue,
		USE_RADIAL_DISTANCES);

	// Create OpenCV images for the intensity and confidence components.
	// Attention: The following constructors don't copy the data from the grab results'
	// resp. component's grab buffers to the OpenCV Mat objects. This means the Mat objects are
	// only valid as long as the grab result smart pointer they are attached to isn't destroyed. If the data
	// is to be used further, the data must be copied, e.g. by calling cv::Mat::clone().
	cv::Mat intensityImage((int)height, (int)width, CV_16UC1, (void*)intensityComponent.GetData());
	cv::Mat confidenceImage((int)height, (int)width, CV_16UC1, (void*)confidenceComponent.GetData());
	
	std::ostringstream filenameDepth;
	filenameDepth << m_nPointCloudsSaved << "Depth.png";
	//std::cout << "Saving " << filenameDepth.str() << std::endl;
	std::ostringstream filenameDepthcolor;
	filenameDepthcolor << m_nPointCloudsSaved << "Depthcolor.png";
	std::ostringstream filenameIntensity;
	filenameIntensity << m_nPointCloudsSaved << "Intensity.png";
	std::ostringstream filenameConfidence;
	filenameConfidence << m_nPointCloudsSaved << "Confidence.png";

	cv::imwrite(filenameDepth.str(), depthImage);
	cv::imwrite(filenameDepthcolor.str(), depthImageColor);
	cv::imwrite(filenameIntensity.str(), intensityImage);
	cv::imwrite(filenameConfidence.str(), confidenceImage);

	/*cv::imshow("Depth", depthImage);
	cv::imshow("Depth (color)", depthImageColor);
	cv::imshow("Intensity", intensityImage);
	cv::imshow("Confidence", confidenceImage);*/
	//cv::waitKey(1);
	return ptrPointCloud;
}

// Saves acquired point clouds.
bool Sample::onImageGrabbed(const CPylonDataContainer& container)
{
	bool success = true;

	// Convert grab result to PCL point cloud.
	PointCloudPtr ptrPointCloud = convertGrabResultToPointCloud(container);
	// Write point cloud to file.
	std::ostringstream filename;
	filename << m_nPointCloudsSaved << ".pcd";
	std::cout << "Saving " << filename.str() << std::endl;
	//std::string saved_pcd_path = "./data/"+ std::to_string(m_nPointCloudsSaved)+"/pointcloud.pcd";
	const auto t0 = std::chrono::high_resolution_clock::now();

	success = -1 != pcl::io::savePCDFileBinary(filename.str(), *ptrPointCloud);

	if (!success)
	{
		std::cerr << "Failed to save " << std::to_string(m_nPointCloudsSaved) << std::endl;
	}
	else
	{
		std::cout << "Saving Succeed" << std::to_string(m_nPointCloudsSaved) << std::endl;
	}
	//m_nPointCloudsSaved++;

	return success;// && m_nPointCloudsSaved < c_countOfPointCloudsToSave
}



int Sample::run()
{
	try
	{
		// Open the first available blaze camera, i.e., establish a connection to the camera device.
		m_camera.Attach(
			CTlFactory::GetInstance().CreateFirstDevice(CDeviceInfo().SetDeviceClass(BaslerGenTlBlazeDeviceClass)));

		// By registering the CBlazeDefaultConfiguration, depth, intensity, and confidence data will be delivered.
		// Depth data is represented as point clouds.
		m_camera.RegisterConfiguration(new CBlazeDefaultConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
		m_camera.Open();
		std::cout << "Connected to camera " << m_camera.GetDeviceInfo().GetFriendlyName() << std::endl;

		// PCL point clouds expect a point's coordinates set to NaN when there is no depth information available
        // for that point.
        // Configure the camera to use NaNs to mark missing depth data.
		//for (auto axis : { Scan3dCoordinateSelector_CoordinateA,
		//				  Scan3dCoordinateSelector_CoordinateB,
		//				  Scan3dCoordinateSelector_CoordinateC })
		//{
		//	// Choose axis to configure.
		//	m_camera.Scan3dCoordinateSelector.SetValue(axis);
		//	// Set value used to mark missing depth data.
		//	m_camera.Scan3dInvalidDataValue.SetValue(std::numeric_limits<float>::quiet_NaN());
		//}
		

		m_camera.Scan3dCoordinateSelector.SetValue(Scan3dCoordinateSelector_CoordinateC);
		m_camera.Scan3dInvalidDataValue.SetValue((double)m_invalidDataValue);


		m_camera.DepthMin.SetValue(0);
		m_camera.DepthMax.SetValue(600);
		m_minDepth = (int)m_camera.DepthMin.GetValue();
		m_maxDepth = (int)m_camera.DepthMax.GetValue();
		m_camera.OperatingMode.SetValue(OperatingMode_LongRange);
		m_camera.ExposureTime.SetValue(50);
		// This smart pointer will receive the grab result data.
		CGrabResultPtr ptrGrabResult;

		for (int k = 0; k < 20; k++)
		{
			m_camera.StartGrabbing();
			for (int i = 0; i < 2; ++i)
			{
				// Wait for an image and then retrieve it. A timeout of 1000 ms is used.
				m_camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
			}
			m_camera.StopGrabbing();

			if (ptrGrabResult->GrabSucceeded())
			{
				// Access the data.
				
				auto container = ptrGrabResult->GetDataContainer();

				onImageGrabbed(container);
				m_nPointCloudsSaved++;
				std::cout << "Press 'q' to exit." << std::endl;
				//loadPointClouds();
			}
			else
			{
				std::cerr << "Failed to grab an image." << std::endl;
			}

		}

		// Clean-up
		m_camera.Close();

	}
	catch (const GenICam::GenericException& e)
	{
		std::cerr << "Exception occurred: " << e.GetDescription() << std::endl;

		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}


