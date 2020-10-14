#include "NaiveApproach.h"


/*Constructor*/
Naive::Naive(int window_size, Mat& image1, Mat& image2, int dmin, double focal_length, double baseline) {
	this->winSize = window_size;	
	this->img1 = image1.clone();
	this->img2 = image2.clone();
	this->dmin = dmin;
	this->focal_length = focal_length;
	this->baseline = baseline;
}

/*Naive stereo matching methods*/
void Naive::NaiveMatchingCalculation() {
	int choice = 1;
	cout << "Naive stereo matching techniques:" << endl;
	cout << "1. Sum of squared intensity difference" << endl;
	cout << "2. Sum of absolute intensity difference" << endl;
	cout << "3. Normalised cross correlation" << endl;
	cout << "4. Normalised SSD" << endl;
	cout << "Please enter your preference" << endl;
	cin >> choice;

	switch (choice) {
		case 1:
			this->NaiveMatching_SSD();
			break;
		case 2:
			this->NaiveMatching_SAD();
			break;
		case 3:
			this->NaiveMatching_NormalisedCrossCorrelation();
			break;
		case 4:
			this->NaiveMatching_NormalisedSSD();
			break;
		default:
			cout << "Enter valid choice" << endl;
			break;
	}
}

void Naive::NaiveMatching_SAD() {

}

void Naive::NaiveMatching_NormalisedSSD() {

}

void Naive::NaiveMatching_NormalisedCrossCorrelation() {

}

void Naive::NaiveMatching_OpenCV() {

}

/*sum of squared intensity difference technique*/
void Naive::NaiveMatching_SSD() {
	Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
	for (int r = this->winSize/2; r < img1.rows - this->winSize/2; r++) {
		for (int c = this->winSize/2; c < img1.cols - this->winSize/2; c++) {
			/*for each horizontal disparity*/
			int minSSd = INT_MAX;
			int disparity = 0.;
			//for (int d = 0; d < img1.cols - this->winSize; d++) {
			for(int d = -c + this->winSize / 2; d < img1.cols - c - this->winSize / 2; ++d){
				/*window around the epipolar line for both the images*/
				int ssd = 0;
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						ssd += pow((img1.at<uchar>(r + i, c + j) - img2.at<uchar>(r + i, c + j + d)), 2);
					}
				}
				if (ssd < minSSd) {
					minSSd = ssd;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	string fileName = "NaiveMatching_SSD.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

void Naive::saveDisparityImage(string& fileName, Mat& disparity) {
	imwrite(fileName, disparity);
}
