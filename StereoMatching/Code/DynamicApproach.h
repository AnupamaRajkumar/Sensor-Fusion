#pragma once
#ifndef  __DYNAMIC__
#define __DYNAMIC__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <omp.h>
#include <Windows.h>

class Dynamic {
public:
	Dynamic(cv::Mat& image1, cv::Mat& image2, int dmin, int window_size, double weight);
	void DynamicApproachCalculation();

private:
	int occlusion, winSize, dmin;
	cv::Mat C, M, img1, img2;

	void FindMinimumCostPath(std::vector<uchar>& DSI);
	void ReconstructOptimalPath(int r, cv::Mat& naive_disparities);
	std::vector<uchar> DisparitySpaceImage(int r);
	void saveDisparityImage(std::string& fileName, cv::Mat& disparity);

	enum MatMat
	{
		Match = 0,				
		LeftOcc = 1,
		RightOcc = 2
	};
};

#endif // ! __DYNAMIC__

