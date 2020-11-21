/*  
Subtask 1: Implement bilateral filter/ bilateral median filter
- evaulate it on an image of choice
- choose 4 different level of sigmas
- test all combinations , 16 in total by running bilateral filtering
Subtask 2: Convert bilateral filter to guided joint bilateral filter for guided image upsampling
- apply filter to upsample the depth image, guided by an RGB image
*/


#include <iostream>
#include "Bilateral.h"

int main(int argc, char** argv)
{
	if (argc < 4) {
		cerr << "Enter the name/path of input image for bilateral filtering, guided image, downsampled image" << endl;
		return -1;
	}
	Mat im = cv::imread(argv[1], 0);
	Mat guidedImg = cv::imread(argv[2], 0);
	Mat downImg = cv::imread(argv[3], 0);

	if (im.data == nullptr || guidedImg.data == nullptr || downImg.data == nullptr) {
		std::cerr << "One of the images failed to load image" << std::endl;
		exit(0);
	}

	/*Add gaussian noise*/
	cv::Mat noise(im.size(), im.type());
	Mat noiseImg = im.clone();
	uchar mean = 0;
	uchar stddev = 25;
	cv::randn(noise, mean, stddev);
	noiseImg = im + noise;

	/*cv::imshow("Original image", im);
	cv::imshow("Noisy image", noiseImg);
	cv::imwrite("Noisy.png", noiseImg);
	cv::imshow("Guided image", guidedImg);
	cv::imshow("Downsampled image", downImg);
	waitKey(0);*/
	int kSize = 0;
	cout << "Enter the kernel size" << endl;
	cin >> kSize;

	Bilateral bilateral(im, noiseImg, guidedImg, downImg, kSize);

	int choice = 1;
	while (choice != 0) {
		cout << "Bilateral Menu" << endl;
		cout << "1. Bilateral filtering" << endl;
		cout << "2. Guided image upsampling using guided joint bilateral filter" << endl;
		cout << "Press 0 to exit" << endl;
		cout << "Enter your choice (1/2)" << endl;
		cin >> choice;

		switch (choice) {
		case 1:
			bilateral.BilateralFilteringMenu();
			break;
		case 2:
			bilateral.UpsamplingMenu();
			break;
		default:
			cout << "Enter a valid choice" << endl;
			break;
		}		
	}

	cv::waitKey();
	return 0;
}
