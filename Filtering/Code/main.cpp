/***********************************************************************
Implementing Conventional Filters and comparing performances
1. Mean/Box filter - OpenCV/own
2. Gaussian Filter - OpenCV/own
3. Median Filter - OpenCV/own
4. Bilateral filter - OpenCV
5. Performance comparison -  SSD, RMSE, PSNR, SSIM, MAE
*************************************************************************/


#include <iostream>
#include <opencv2/opencv.hpp>

#include "Filters.h"

using namespace std;
using namespace cv;


int main(int argc, char** argv) {

	if (argc < 2) {
		cout << "Input image not found" << endl;
		cout << "Press enter to exit" << endl;
		cin.get();
		return -1;
	}
	Mat im = imread(argv[1], IMREAD_GRAYSCALE);

	if (im.data == nullptr) {
		std::cerr << "Failed to load image" << std::endl;
		exit(0);
	}

	imshow("Original", im);

	/*add gaussian noise to the input image*/
	cv::Mat noise(im.size(), im.type());
	cv::Mat noiseImg = im.clone();
	uchar mean = 0;
	uchar stddev = 25;
	cv::randn(noise, mean, stddev);

	noiseImg = im + noise;
	cv::imshow("Noisy Image", noiseImg);
	imwrite("Noisy.png", noiseImg);

	Filters filt;

	filt.DenoiseImage(im, noiseImg);

	cv::waitKey();
	return 0;
}