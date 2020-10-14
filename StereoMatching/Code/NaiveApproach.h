#pragma once
#ifndef __NAIVE__
#define __NAIVE__

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class Naive {
public:
	Naive(int window_size, Mat& image1, Mat& image2, int dmin, double focal_length, double baseline);
	void NaiveMatchingCalculation();
	void NaiveMatching_OpenCV();			//Opencv stereo matching methods

private:
	int winSize, dmin;
	Mat img1, img2;
	double focal_length, baseline;

	void NaiveMatching_SSD();			//sum of squared intensity difference
	void NaiveMatching_SAD();			//sum of absolute intensity difference
	void NaiveMatching_NormalisedCrossCorrelation();	//normalised cross correlation
	void NaiveMatching_NormalisedSSD();		//normalised sum of squared intensity difference
	void saveDisparityImage(string& fileName, Mat& disparity);

};
#endif // !__NAIVE__

