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

	void Disparity2PointCloud(
		const std::string& output_file,
		int height, int width, cv::Mat& disparities,
		const int& window_size,
		const int& dmin, const double& baseline, const double& focal_length);

};


#endif // !__UTILITY__

