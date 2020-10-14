#pragma once
#ifndef __UTILITY__
#define __UTILITY__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <ppl.h>

using namespace cv;
using namespace std;

class Utility {
public:
	void StereoEstimation_Naive(
		const int& window_size,
		const int& dmin,
		cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale);

	void StereoEstimation_NaiveFast(
		const int& window_size,
		const int& dmin,
		cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale);

	void Disparity2PointCloud(
		const std::string& output_file,
		int height, int width, cv::Mat& disparities,
		const int& window_size,
		const int& dmin, const double& baseline, const double& focal_length);

};


#endif // !__UTILITY__

