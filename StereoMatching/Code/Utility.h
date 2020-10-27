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
	void Disparity2PointCloud(std::string& output_file, cv::Mat& disparities, int& window_size, 
							  int& dmin, double& baseline, double& focal_length, double& cx_d, double& cy_d, double doffs);
	void saveDisparityImage(std::string& fileName, cv::Mat& disparity);
	void ConvertPixelToMetric(cv::Mat& disparities, cv::Mat& pxToMetric, int dmin, 
							  double baseline, double focal_length, double doffs);

};


#endif // !__UTILITY__

