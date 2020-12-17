#pragma once
#ifndef  __PCLREGIS__
#define __PCLREGIS__

#define _USE_MATH_DEFINES
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <nanoflann.hpp>
#include <fstream>
#include <omp.h>
#include <chrono>
#include <math.h>

using namespace std;
using namespace cv;
using namespace nanoflann;
using namespace Eigen;

#define SVD_REGISTRATION 1			 /*Only one of these registration techniques */
#define QUAT_REGISTRATION 0			 /*should be activated at a time*/

class CloudRegistration {
public:
	CloudRegistration(char* modelPCLFile, char* dataPCLFile);

private:
	char* modelPCLFile;
	char* dataPCLFile;
	vector<pair<int, size_t>> nearestPts;
	int maxIterations = 20;
	double minThreshold = 0.0001;
	double error = 0.0;
	Mat Rotation, Translation;

	typedef struct Point {
		Point3d dataPt, normalPt;
	};

	/*point cloud structure as needed by nanoflann*/
	typedef struct PointCloud
	{
		std::vector<Point>  pts;

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return pts.size(); }

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			if (dim == 0) return pts[idx].dataPt.x;
			else if (dim == 1) return pts[idx].dataPt.y;
			else return pts[idx].dataPt.z;
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

	}allPtCloud;

	/*model and data point cloud variables*/
	allPtCloud modelPCL, dataPCL;
	typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, allPtCloud>,
		allPtCloud, 3> myKDTree;
	myKDTree *KDTree;

	void LoadData(allPtCloud& points, char* fileName);
	void IterativeClosestPoint();
	size_t FindNearestNeighbor(Point& queryPt, allPtCloud& modelPCL);
	void CalculateTransformationMatrix();
	void WriteDataPoints(allPtCloud& points, string fileName);
	double CalculateDistanceError();
	void AddNoiseToData(allPtCloud& dataPCL);
};

#endif // ! __PCLREGIS__

