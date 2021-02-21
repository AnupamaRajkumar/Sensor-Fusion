#pragma once
#ifndef __PCL__
#define __PCL__

#include "Utils.h"
#include <nanoflann.hpp>

using namespace nanoflann;

class CloudClustering {
public:
	CloudClustering(char* fileName);
	void CloudClusteringMenu();

private:
	char* fileName;
	double clusterThreshold;
	int minClusterSize;
	int numOfClusters;
	int iterations;

	/*point cloud structure as needed by nanoflann*/
	typedef struct PointCloud
	{
		std::vector<Utils::PointXYZRGB>  pts;

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return pts.size(); }

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			if (dim == 0) return pts[idx].point.x;
			else if (dim == 1) return pts[idx].point.y;
			else return pts[idx].point.z;
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

	}allPtCloud;
	/*point cloud variables*/
	allPtCloud clusterPtCloud;
	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, allPtCloud>, allPtCloud, 3> myKDTree;

	/*Clustering Algorithms*/
	void EuclideanClusteringBasedOnRadius(std::vector<Utils::PointXYZRGB>& points);
	void EuclideanClusteringBasedOnKNN(std::vector<Utils::PointXYZRGB>& points);
	void KMeansClustering(std::vector<Utils::PointXYZRGB>& points);
};
#endif // !__PCL__

