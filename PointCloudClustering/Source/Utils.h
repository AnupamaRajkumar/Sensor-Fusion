#pragma once
#ifndef __UTILS__
#define __UTILS__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <cstdlib>
#include <random>

class Utils {
public:
	typedef struct points {
		cv::Point3d point;	//3d point coordinates
		int red;			//red value
		int green;			//green value
		int blue;			//blue value
		int centroidId;		//this parameter is used to determine the centroid that a point
							//belongs to in k-mean clustering
	}PointXYZRGB;			//Structure for points in the point clouds

	//Function to load the point cloud from point cloud file
	void LoadData(std::vector<PointXYZRGB>& points, char*fileName);
	//Function to write the point cloud after desired operation into filename
	void WriteDataPoints(std::vector<PointXYZRGB>& points, std::string fileName);
	//Function to generate colors for different clusters
	cv::Point3d PointColorAssignment();
	//Function to calculate distance between two points
	double Distance(cv::Point3d x, cv::Point3d y);
};
#endif // !__UTILS__

