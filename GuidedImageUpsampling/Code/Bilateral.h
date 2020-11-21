#pragma once
#ifndef __BILATERAL__
#define __BILATERAL__

#include <iostream>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

class Bilateral {

public:
	Bilateral(Mat& origImg, Mat& noisyImg, Mat& guidedImg, Mat& downImg, int kSize);
	void UpsamplingMenu();
	void BilateralFilteringMenu();
private:
	int kSize;
	Mat origImg, noisyImg, guidedImg, downImg;
	
	float CalculateKernelBilateralFilter(float distance, float sigma);
	float distance(int currentX, int currentY, int neighborX, int neighborY);
	void CalculateBilateralFiltering();
	Mat BilateralImplementation(float sigmaSpat, float sigmaRad);
	void CalculateBilateralMedianFiltering();
	Mat BilateralMedianFiltering(float sigmaSpat, float sigmaRad);
	void StraighforwardUpsampling();
	void IterativeUpsampling();
	Mat JointBilateralFiltering(Mat& highRes, Mat& loRes, float sigmaSpat, float sigmaRad);
	
};


#endif // !__BILATERAL__

