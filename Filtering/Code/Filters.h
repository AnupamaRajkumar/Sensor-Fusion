#pragma once
#ifndef __FILTERS__
#define __FILTERS__

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Filters {
public:
	Filters();
	void DenoiseImage(Mat& origImg, Mat& noiseImg);

private:
	int kSize;
	int stdDevX;
	int stdDevY;
	int meanX;
	int meanY;

	void PerformanceMetrics(Mat& origImg, Mat& denoisedImg);
	float SSIMCalculation(Mat& origImg, Mat& denoisedImg);
	Mat BoxFilter(Mat& img);
	Mat GaussFilter(Mat& img);
	Mat MedianFilter(Mat& img);
	Mat GaussianKernel2D();
};
#endif // !__FILTERS__

