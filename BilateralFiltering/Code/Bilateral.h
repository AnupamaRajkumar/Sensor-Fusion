#pragma once

#ifndef __BILATERAL__
#define __BILATERAL__

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Bilateral {
public:
	Bilateral();
	void BilateralFilteringMenu(Mat& noisyImg, Mat& imFlash, Mat& imNoFlash);
private:
	int kSize;
	float sigma_spatial;
	float sigma_radiometric;

	Mat OpenCVBilateral(Mat& noisyImg);
	Mat BilateralImplementation_1(Mat& noisyImg);
	Mat GaussianKernel2D();
	float p(float diff);
	Mat BilateralImplementation_2(Mat& noisyImg);
	float CalculateKernelBilateralFilter(float distance, float sigma);
	float distance(int currentX, int currentY, int neighborX, int neighborY);
	Mat JointBilateralUpsampling_2(Mat& imFlash, Mat& imNoFlash);
	Mat JointBilateralUpsampling_1(Mat& imFlash, Mat& imNoFlash);
	Mat BilateralMedianFiltering(Mat& noisyImg);
};

#endif // !__BILATERAL__

