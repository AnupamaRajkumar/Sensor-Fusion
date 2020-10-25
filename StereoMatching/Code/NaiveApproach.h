#pragma once
#ifndef __NAIVE__
#define __NAIVE__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <omp.h>
#include <Windows.h>
#include <opencv2/calib3d.hpp>


class Naive {
public:
	Naive(int window_size, cv::Mat& image1, cv::Mat& image2, int dmin, double focal_length, double baseline);
	void NaiveMatchingCalculation();
	void NaiveMatching_OpenCV();			//Opencv stereo matching methods

private:
	int winSize, dmin, dmax;
	cv::Mat img1, img2;
	double focal_length, baseline;

	/*http://www.cs.cmu.edu/~16385/s17/Slides/13.2_Stereo_Matching.pdf*/
	void NaiveMatching_SSD();									//sum of squared intensity difference
	void NaiveMatching_NormalisedSSD();							//Normalised sum of squared differences
	void NaiveMatching_SAD();									//sum of absolute intensity difference
	void NaiveMatching_ZSAD();									//zero mean sum of absolute intensity difference
	void NaiveMatching_LSSAD();									//locally scaled sum of squared intensity difference
	void NaiveMatching_NormalisedCrossCorrelation();			//normalised cross correlation
	void NaiveMatching_CrossCorrelation();						//normalised sum of squared intensity difference

};
#endif // !__NAIVE__

