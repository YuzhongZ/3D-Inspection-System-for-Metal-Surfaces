
#include <pcl/search/kdtree.h>

#include "Point.h"

void flatness(int sample_count, PointCloudPtrxyz transformed_cloud0, std::vector<Eigen::Vector2f> u_v)
{
	int samplesize = sample_count;

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrcloudrandom(new pcl::PointCloud<pcl::PointXYZ>);
	ptrcloudrandom->resize(samplesize);
	for (uint32_t i = 0; i < samplesize; ++i)
	{
		//Eigen::Vector2f u_v(float(i) / float(sample_count), RadicalInverse_VdC(i));
		ptrcloudrandom->points[i].x = (u_v[i](0) - 0.5) * 176;//(edge_right - edge_left - 50);
		ptrcloudrandom->points[i].y = (u_v[i](1) - 0.5) * 150;// (edge_yright - edge_yleft - 50);
		ptrcloudrandom->points[i].z = 0;

	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr addcloud(new pcl::PointCloud<pcl::PointXYZ>);
	*addcloud = (*ptrcloudrandom) + (*transformed_cloud0);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3(new pcl::search::KdTree<pcl::PointXYZ>);
	tree3->setInputCloud(addcloud);
	int K = 17;
	std::vector<int> pointIdxKNNSearch;
	std::vector<float> pointKNNSquaredDistance;

	for (size_t i = 0; i < ptrcloudrandom->size(); i++)
	{

		float sumz = 0;
		float averz = 0;
		if (tree3->nearestKSearch(ptrcloudrandom->points[i], K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
		{
			for (std::size_t j = 0; j < pointIdxKNNSearch.size() - 1; ++j)
			{
				sumz += addcloud->points[pointIdxKNNSearch[j + 1]].z;
			}
			averz = sumz / (pointIdxKNNSearch.size() - 1);
			ptrcloudrandom->points[i].z = averz;

		}
	}


	///计算主方向
	//Eigen::Vector4f centroidpca_target;							
	//pcl::compute3DCentroid(*ptrcloudrandom, centroidpca_target);	

	//Eigen::Matrix3f covariance_target;
	//computeCovarianceMatrixNormalized(*ptrcloudrandom, centroidpca_target, covariance_target);		

	//Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver_target(covariance_target, Eigen::ComputeEigenvectors);
	//Eigen::Matrix3f eigen_vectors_target = eigen_solver_target.eigenvectors();
	//Eigen::Vector3f eigen_values_target = eigen_solver_target.eigenvalues();
	//Eigen::Vector3f::Index minRow, minCol;
	//eigen_values_target.minCoeff(&minRow, &minCol);

	//Eigen::Vector3f normal = eigen_vectors_target.col(minCol);
	//float D = -normal.dot(centroidpca_target.head<3>());

	//cout << "平面模型系数为：\n"
	//	<< "A=" << normal[0] << "\n"
	//	<< "B=" << normal[1] << "\n"
	//	<< "C=" << normal[2] << "\n"
	//	<< "D=" << D << "\n" << endl;


	float dist_plane = 0;
	float dist_planemax = -100;
	float dist_planemin = 100;
	float p = 0;
	float q = 0;
	float u = 0;
	float w = 0;
	float v = 0;
	float sumz = 0;
	float sumxz = 0;
	float sumyz = 0;
	std::vector<float> distpointplane;
	pcl::PointXYZ dismaxpoint, disminpoint;
	for (int i = 0; i < ptrcloudrandom->size(); i++)
	{
		p += ptrcloudrandom->points[i].x;
		q += ptrcloudrandom->points[i].y;
		u += ptrcloudrandom->points[i].x*ptrcloudrandom->points[i].x;
		v += ptrcloudrandom->points[i].y*ptrcloudrandom->points[i].y;
		w += ptrcloudrandom->points[i].x*ptrcloudrandom->points[i].y;
		sumz += ptrcloudrandom->points[i].z;
		sumxz += ptrcloudrandom->points[i].x*ptrcloudrandom->points[i].z;
		sumyz += ptrcloudrandom->points[i].y*ptrcloudrandom->points[i].z;

	}
	float S = samplesize * u*v + 2 * p*w*q - u * q*q - v * p*p - samplesize * w*w;
	float C = ((u * v - w * w)*sumz + (q * w - p * v)*sumxz + (p * w - q * u)*sumyz) / S;
	float A = ((q * w - p * v)*sumz + (samplesize * v - q * q)*sumxz + (p * q - samplesize * w)*sumyz) / S;
	float B = ((p * w - u * q)*sumz + (p * q - samplesize * w)*sumxz + (samplesize * u - p * p)*sumyz) / S;
	//cout << "A:" << A << "  " << "B:" << B << "  " << "C:" << C << endl;

	for (int i = 0; i < ptrcloudrandom->size(); i++)
	{
		dist_plane = (ptrcloudrandom->points[i].z - (A*ptrcloudrandom->points[i].x + B * ptrcloudrandom->points[i].y + C)) / sqrt(1 + A * A + B * B);//pcl::pointToPlaneDistanceSigned(ptrcloudrandom->points[i], A, B, -1, C);
		distpointplane.push_back(dist_plane);
		if (dist_plane > dist_planemax)
		{
			dist_planemax = dist_plane;
			dismaxpoint = ptrcloudrandom->points[i];
		}
		else if (dist_plane < dist_planemin)
		{
			dist_planemin = dist_plane;
			disminpoint = ptrcloudrandom->points[i];
		}
	}
	//sort(distpointplane.begin(), distpointplane.end());
	//float flatness = distpointplane.back() - distpointplane.front();
	float flatness = dist_planemax - dist_planemin;
	cout << "flatness:" << flatness << endl;
	float T = 0.2;
	float u1f = (sqrt(2)*T) / sqrt(samplesize);//( 6*sqrt(samplesize))
	float u2f = 5 / 3;
	float u3f = 1 / 3;
	float u0f = sqrt(u1f*u1f + u2f * u2f + u3f * u3f);
	float uA = ((q * w - p * v) + (samplesize * v - q * q)*p + (p * q - samplesize * w)*q) * u0f / S;
	float uB = ((p * w - u * q) + (p * q - samplesize * w)*p + (samplesize * u - p * p)*q) *u0f / S;
	float fzm = 1 / sqrt(1 + A * A + B * B);
	float fzL = -fzm;
	float fa = (disminpoint.x - dismaxpoint.x) / sqrt(1 + A * A + B * B) - A * (dismaxpoint.z - disminpoint.z - A * (dismaxpoint.x - disminpoint.x) - B * (dismaxpoint.y - disminpoint.y)) / sqrt((1 + A * A + B * B)*(1 + A * A + B * B)*(1 + A * A + B * B));
	float fb = (disminpoint.y - dismaxpoint.y) / sqrt(1 + A * A + B * B) - B * (dismaxpoint.z - disminpoint.z - A * (dismaxpoint.x - disminpoint.x) - B * (dismaxpoint.y - disminpoint.y)) / sqrt((1 + A * A + B * B)*(1 + A * A + B * B)*(1 + A * A + B * B));
	float uf = sqrt((fzm*u0f)*(fzm*u0f) + (fzL*u0f)*(fzL*u0f) + (fa*uA)*(fa*uA) + (fb*uB)*(fb*uB));
	//cout << "uf:" << uf << endl;
	//auto t7 = chrono::steady_clock::now();
	//auto dt7 = chrono::duration_cast<chrono::duration<double>>(t7 - t1).count();
	//fout <<flatness << " "<< uf <<endl;
	cout << flatness << " " << uf << endl;

}
