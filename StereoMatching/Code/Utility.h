#pragma once
#ifndef __UTILITY__
#define __UTILITY__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <ppl.h>

class Utility {
public:
	void Disparity2PointCloud(std::string& output_file, int height, int width, cv::Mat& disparities, int& window_size, 
							  int& dmin, double& baseline, double& focal_length);
	void saveDisparityImage(std::string& fileName, cv::Mat& disparity);

};


#endif // !__UTILITY__

