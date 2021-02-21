#include "PointCloudClustering.h"

Utils utility;

/***************CloudClustering Constructor*****************************/
CloudClustering::CloudClustering(char* fileName) {
	this->fileName = fileName;				//name of the input point cloud file
	this->clusterThreshold = 40.;			//thereshold for radius based neighbors determination
	this->minClusterSize = 1500;			//cluster size, k value for nearest neighbor
	this->iterations = 5;					//number of iterations for k-mean clustering
	this->numOfClusters = 10;				//number of clusters in k-mean clustering
	this->CloudClusteringMenu();			//Invoking menu for cloud clustering
}

/**************************************************************************
Function Name    : CloudClusteringMenu
Parameters		 : void
Returns			 : void
Function		 : Menu for different cloud clustering methods
Author			 : Anupama Rajkumar
****************************************************************************/
void CloudClustering::CloudClusteringMenu() {
	
	std::vector<Utils::PointXYZRGB> points, clusterPointSet;
	int choice = 1;
	/*1. Load the point clouds from the point cloud file*/
	utility.LoadData(points, this->fileName);
	/*2. Clustering algorithm - Euclidean Clustering using radius and nearest neighbor
		, k-means clustering*/
	std::cout << "Point cloud clustering menu:" << std::endl;
	std::cout << "1. Clustering based on Radius" << std::endl;
	std::cout << "2. Clustering based on Nearest Neighbors" << std::endl;
	std::cout << "3. Clustering based on k-means" << std::endl;
	std::cout << "Please enter your choice" << std::endl;
	std::cin >> choice;
	switch (choice) {
	case 1:
		this->EuclideanClusteringBasedOnRadius(points);
		break;
	case 2:
		this->EuclideanClusteringBasedOnKNN(points);
		break;
	case 3:
		this->KMeansClustering(points);
		break;
	default:
		std::cerr << "Please enter valid choice (1/2/3)" << std::endl;
		break;
	} 
}


/******************************************************************************
Function Name    : EuclideanClusteringBasedOnRadius
Parameters		 : vector of points
Returns			 : void
Function		 : Function for finding closest point based on radius search
Author			 : Anupama Rajkumar
******************************************************************************/
void CloudClustering::EuclideanClusteringBasedOnRadius(std::vector<Utils::PointXYZRGB>& points) {
	std::vector<std::vector<Utils::PointXYZRGB>> clusters;
	clusterPtCloud.pts = points;
	/*Create a kd-tree representation of the input point cloud dataset*/
	myKDTree index(3, clusterPtCloud, KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();
	std::vector<Utils::PointXYZRGB> clusterQueue;
	/*for each point in the point cloud*/
	for (int i = 0; i < points.size(); i++) {	//clusterPtCloud.pts == points
		/*check if the point is processed already or not. Pick those points that have not been assigned
		   to any cluster yet*/
		/*ref: https://github.com/jlblancoc/nanoflann/blob/master/examples/pointcloud_kdd_radius.cpp */
		if ((points[i].red == 0) && (points[i].green == 0) && (points[i].blue == 0)) {
			double queryPt[3] = { points[i].point.x, points[i].point.y, points[i].point.z };

			/*search radius within which the points form a cluster*/
			const double search_radius = static_cast<double>(this->clusterThreshold);
			std::vector<std::pair<size_t, double> > ret_matches;

			nanoflann::SearchParams params;
			/*number of points that lie within the specified radius*/
			const size_t nMatches = index.radiusSearch(&queryPt[0], search_radius, ret_matches, params);

			/************debugging******************/
			//std::cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches << " matches\n";
			//for (size_t i = 0; i < nMatches; i++)
			//	std::cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i << "]="
			//	<< ret_matches[i].second << std::endl;
			//std::cout << "\n";
			/************debugging******************/

			/*Assign a color to this cluster*/
			cv::Point3d color = utility.PointColorAssignment();
			for (size_t s = 0; s < nMatches; s++) {
				/*for all the determined points within radius, if the points have not been assigned to
				a cluster yet, assign to the current cluster queue*/
				if ((points[ret_matches[s].first].red == 0) && 
					(points[ret_matches[s].first].green == 0) && 
					(points[ret_matches[s].first].blue == 0)) {
						points[ret_matches[s].first].red	= color.x;
						points[ret_matches[s].first].green	= color.y;
						points[ret_matches[s].first].blue	= color.z;
						clusterQueue.emplace_back(points[ret_matches[s].first]);
				}
			}
			/*points added to cluster*/
			clusters.emplace_back(clusterQueue);
			clusterQueue.clear();
		}
		else {
			continue;
		}
	}
	//write cluster points to file
	std::cout << "Number of clusters found:" << clusters.size() << std::endl;
	std::string fileName = "CloudClusteringBasedOnRadius.txt";
	utility.WriteDataPoints(points, fileName);
}

/********************************************************************************************
Function Name    : EuclideanClusteringBasedOnRadius
Parameters		 : vector of points
Returns			 : void
Function		 : Function for finding closest points based on k - nearest neighbors
Author			 : Anupama Rajkumar
**********************************************************************************************/
void CloudClustering::EuclideanClusteringBasedOnKNN(std::vector<Utils::PointXYZRGB>& points) {
	std::vector<std::vector<Utils::PointXYZRGB>> clusters;
	clusterPtCloud.pts = points;
	/*Create a kd-tree representation of the input point cloud dataset*/
	myKDTree index(3, clusterPtCloud, KDTreeSingleIndexAdaptorParams(10));
	index.buildIndex();
	std::vector<Utils::PointXYZRGB> clusterQueue;
	for (int i = 0; i < points.size(); i++) {
		/*check if the point is processed already or not. Pick those points that have not been assigned
		  to any cluster yet*/
		/* ref : https://github.com/jlblancoc/nanoflann/blob/master/examples/pointcloud_kdd_radius.cpp */
		if ((points[i].red == 0) && (points[i].green == 0) && (points[i].blue == 0)) {
			double queryPt[3] = { points[i].point.x, points[i].point.y, points[i].point.z };

			/*set the number of nearest neighbors ie. k in knn search*/
			size_t nearestNeighbors = this->minClusterSize;
			std::vector<size_t>   ret_index(nearestNeighbors);
			std::vector<double> sqDist(nearestNeighbors);
			/*find the k nearest neighbors*/
			nearestNeighbors = index.knnSearch(&queryPt[0], nearestNeighbors, &ret_index[0], &sqDist[0]);

			// In case of less points in the tree than requested:
			ret_index.resize(nearestNeighbors);
			sqDist.resize(nearestNeighbors);

			//std::cout << "Nearest Neighbors: " << nearestNeighbors << std::endl;

			/*******************************debugging*************************************/
			//std::cout << "knnSearch(): num_results=" << nearestNeighbors << "\n";
			//for (size_t i = 0; i < nearestNeighbors; i++)
			//	std::cout << "idx[" << i << "]=" << ret_index[i] << " dist[" << i << "]=" << sqDist[i] << std::endl;
			//std::cout << "\n";
			/*******************************debugging*************************************/

			/*Assign a color to this cluster*/
			cv::Point3d color = utility.PointColorAssignment();
			for (size_t s = 0; s < nearestNeighbors; s++) {
				/*for all the k neighbors, if they have not been assigned to a cluster yet, assign to
				the current cluster queue*/
				if ((points[ret_index[s]].red == 0) &&
					(points[ret_index[s]].green == 0) &&
					(points[ret_index[s]].blue == 0)) {
					points[ret_index[s]].red	= color.x;
					points[ret_index[s]].green	= color.y;
					points[ret_index[s]].blue	= color.z;
					clusterQueue.emplace_back(points[ret_index[s]]);
				}
			}
			/*point added to cluster*/
			clusters.emplace_back(clusterQueue);
			clusterQueue.clear();
		}
		else {
			continue;
		}
	}
	//write cluster points into a file
	std::cout << "Number of clusters are: " << clusters.size() << std::endl;
	std::string fileName = "ClusteringBasedOnNearestNeighbors.txt";
	utility.WriteDataPoints(points, fileName);
}

/**********************************************************************************
Function Name    : KMeansClustering
Parameters		 : vector of points
Returns			 : void
Function		 : Function for finding closest point based on k-mean clustering
Author			 : Anupama Rajkumar
***********************************************************************************/
void CloudClustering::KMeansClustering(std::vector<Utils::PointXYZRGB>& points) {
	/*1. Start with randomly selected centroids*/
	std::vector<Utils::PointXYZRGB> centroids;
	std::random_device rd;
	std::mt19937 gen(rd());										// seed the generator
	std::uniform_int_distribution<> distr(0, points.size()-1);  // define the range
	/*initialise random clusters and assign color to them*/
	for (int i = 0; i < this->numOfClusters; i++) {
		double val = distr(gen);
		Utils::PointXYZRGB centroid;
		cv::Point3d colors = utility.PointColorAssignment();
		centroid.point = points[val].point;
		centroid.red = colors.x;
		centroid.green = colors.y;
		centroid.blue = colors.z; 
		centroids.emplace_back(centroid);
	}
	std::cout << "Cluster centroids chosen randomly" << std::endl;
	int iter = 0;
	/*2. Assign all the points closest to the cluster centroid*/
	do {
		std::cout << "Calculating centroids in iteration number:" << iter + 1 << std::endl;
		for (int p = 0; p < points.size(); p++) {
			double dist, minDist;
			minDist = std::numeric_limits<double>::max();
			for (int c = 0; c < centroids.size(); c++) {
				dist = utility.Distance(points[p].point, centroids[c].point);
				if (dist < minDist) {
					minDist = dist;
					points[p].centroidId = c;
				}
			}
		}

		/*Update the centroids depending on the mean*/
		for (int c = 0; c < centroids.size(); c++) {
			int centroidCnt = 0;
			double sumX = 0., sumY = 0., sumZ = 0.;
			for (int p = 0; p < points.size(); p++) {
				if (points[p].centroidId == c) {
					centroidCnt += 1;
					sumX += points[p].point.x;
					sumY += points[p].point.y;
					sumZ += points[p].point.z;
				}
			}
			/*updated/adjusted centroids based on mean of the calcualted clusters*/
			centroids[c].point.x = sumX / centroidCnt;
			centroids[c].point.y = sumY / centroidCnt;
			centroids[c].point.z = sumZ / centroidCnt;
		}
		iter++;
	} while (iter < this->iterations); /*Halt when the number of iterations have been achieved*/

	/*update the color of points depending on cluster assigned to them*/
	for (int i = 0; i < points.size(); i++) {
		points[i].red	= centroids[points[i].centroidId].red;
		points[i].green = centroids[points[i].centroidId].green;
		points[i].blue	= centroids[points[i].centroidId].blue;
	}

	/*Write the points to file*/
	std::string fileName = "CentroidClusteringKMeans.txt";
	utility.WriteDataPoints(points, fileName);
}


