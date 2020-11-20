/*****************************************************************
1. Implement Bilateral Filtering
2. Implement Joint Bilateral Filtering and check what it does
3. Bilateral Mean Filter
******************************************************************/


#include <iostream>
#include <opencv2/opencv.hpp>

#include "Bilateral.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv ) {
	if (argc < 4) {
		cerr << "Enter the name/path of input image, flash and no=flash image" << endl;
		return -1;
	}
	cv::Mat im = cv::imread(argv[1], 0);
	Mat imFlash = imread(argv[2], 0);
	Mat imNoFlash = imread(argv[3], 0);

	if (im.data == nullptr) {
	std::cerr << "Failed to load image" << std::endl;
	exit(0);
	}
	/*Add gaussian noise*/
	cv::Mat noise(im.size(), im.type());
	Mat noiseImg = im.clone();
	uchar mean = 0;
	uchar stddev = 25;
	cv::randn(noise, mean, stddev);
	noiseImg = im + noise;

	cv::imshow("Original image", im);
	cv::imshow("Flash image", imFlash);
	cv::imshow("No Flash image", imNoFlash);
	imshow("Noisy image", noiseImg);
	imwrite("Noisy.png", noiseImg);
	imwrite("Flash.png", imFlash);
	imwrite("NoFlash.png", imNoFlash);
	

	Bilateral bilateral(im, noiseImg, imFlash, imNoFlash);

	bilateral.BilateralFilteringMenu();

	cv::waitKey();
	return 0;
}