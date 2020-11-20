#pragma once

#ifndef __BILATERAL__
#define __BILATERAL__

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Bilateral {
public:
	Bilateral(Mat& origImg, Mat& noisyImg, Mat& imFlash, Mat& imNoFlash);
	void BilateralFilteringMenu();
private:
	int kSize;
	float sigma_spatial;
	float sigma_radiometric;
	float sigma_median;
	Mat origImg, noisyImg, imFlash, imNoFlash;

	Mat OpenCVBilateral(Mat& noisyImg);
	Mat BilateralImplementation_1(Mat& noisyImg);
	Mat GaussianKernel2D();
	Mat Lorentzian();
	Mat BoxKernel();
	float p(float diff);
	Mat BilateralImplementation_2(Mat& noisyImg);
	float CalculateKernelBilateralFilter(float distance, float sigma);
	float distance(int currentX, int currentY, int neighborX, int neighborY);
	Mat JointBilateralFiltering_2(Mat& imFlash, Mat& imNoFlash);
	Mat JointBilateralFiltering_1(Mat& imFlash, Mat& imNoFlash);
	Mat BilateralMedianFiltering(Mat& noisyImg);
	float PerformanceMetrics(Mat& origImg, Mat& denoisedImg);
};

#endif // !__BILATERAL__

