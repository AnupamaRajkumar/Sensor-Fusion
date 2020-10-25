#include "NaiveApproach.h"

/*****************************************************************************
Naive approach primarily consists of following steps ie. a typical stereo pipeline
comprises of following steps:
1. Matching cost computation
2. Cost aggregation
3. Disparity computation
4. Disparity refinement - consistency check, hole filling, removing spurious matches etc
**********************************************************************************/

/*Constructor*/
Naive::Naive(int window_size, cv::Mat& image1, cv::Mat& image2, int dmin, double focal_length, double baseline) {
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
	std::cout << "Naive stereo matching techniques:" << std::endl;
	std::cout << "1. Sum of squared intensity difference (SSD)" << std::endl;
	std::cout << "2. Sum of absolute intensity difference (SAD)" << std::endl;
	std::cout << "3. Zero mean SAD (ZSAD)" << std::endl;
	std::cout << "4. Locally scaled SAD (LSSAD)" << std::endl;
	std::cout << "5. Normalised SSD (NSSD)" << std::endl;
	std::cout << "6. Cross correlation" << std::endl;
	std::cout << "7. Normalised cross correlation" << std::endl;
	std::cout << "Please enter your preference" << std::endl;
	std::cin >> choice;

	switch (choice) {
		case 1:
			this->NaiveMatching_SSD();
			break;
		case 2:
			this->NaiveMatching_SAD();
			break;
		case 3:
			this->NaiveMatching_ZSAD();
			break;
		case 4:
			this->NaiveMatching_LSSAD();
			break;
		case 5:
			this->NaiveMatching_NormalisedSSD();
			break;
		case 6:
			this->NaiveMatching_CrossCorrelation();
			break;
		case 7:
			this->NaiveMatching_NormalisedCrossCorrelation();
			break;
		default:
			std::cout << "Enter valid choice" << std::endl;
			break;
	}
}

/*sum of absolute intensity difference technique*/
void Naive::NaiveMatching_SAD() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*for each horizontal disparity*/
			int minSAd = INT_MAX;
			int disparity = 0.;

			for (int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d) {
				/*window around the epipolar line for both the images*/
				int sad = 0;
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						sad += abs((img1.at<uchar>(r + i, c + j) - img2.at<uchar>(r + i, c + j + d)));
					}
				}
				if (sad < minSAd) {
					minSAd = sad;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	std::string fileName = "NaiveMatching_SAD.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

/*https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8570471*/
/*zero mean sum of absolute differences in intensity*/
void Naive::NaiveMatching_ZSAD() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*for each horizontal disparity*/
			int minSAd = INT_MAX;
			int disparity = 0.;

			for (int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d) {
				/*window around the epipolar line for both the images*/
				int sad = 0;
				int mean1, mean2;
				mean1 = 0, mean2 = 0;
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						/*calculate mean of each patch for both the images*/
						mean1 += img1.at<uchar>(r + i, c + j);
						mean1 /= (this->winSize * this->winSize);
						mean2 += img2.at<uchar>(r + i, c + j + d);
						mean2 /= (this->winSize * this->winSize);
						sad += abs(img1.at<uchar>(r + i, c + j) - mean1 - img2.at<uchar>(r + i, c + j + d) + mean2);
					}
				}
				if (sad < minSAd) {
					minSAd = sad;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	std::string fileName = "NaiveMatching_ZSAD.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

/*Locally scaled square of absolute differences (LSSAD)*/
/*http://www.cs.cmu.edu/~16385/s17/Slides/13.2_Stereo_Matching.pdf*/
void Naive::NaiveMatching_LSSAD() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*for each horizontal disparity*/
			int minSAd = INT_MAX;
			int disparity = 0.;
			for (int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d) {
				/*window around the epipolar line for both the images*/
				int sad = 0;
				int mean1, mean2;
				mean1 = 0, mean2 = 1;
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						/*calculate mean of each patch for both the images*/
						mean1 += img1.at<uchar>(r + i, c + j);
						mean1 /= (this->winSize * this->winSize);
						mean2 += img2.at<uchar>(r + i, c + j + d);
						mean2 /= (this->winSize * this->winSize);
						int term1 = img1.at<uchar>(r + i, c + j);
						int term2 = 0;
						if(mean2 > 0)
							term2 = (mean1 / mean2)*img2.at<uchar>(r + i, c + j + d);
						else
							term2 = mean1 * img2.at<uchar>(r + i, c + j + d);
						sad += abs(term1 - term2);
					}
				}
				if (sad < minSAd) {
					minSAd = sad;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	std::string fileName = "NaiveMatching_LSSAD.png";
	this->saveDisparityImage(fileName, naive_disparities);
}


/*sum of squared intensity difference technique*/
void Naive::NaiveMatching_SSD() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize/2; r < this->img1.rows - this->winSize/2; r++) {
		for (int c = this->winSize/2; c < this->img1.cols - this->winSize/2; c++) {
			/*for each horizontal disparity*/
			int minSSd = INT_MAX;
			int disparity = 0.;

			for(int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d){
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
	std::string fileName = "NaiveMatching_SSD.png";
	this->saveDisparityImage(fileName, naive_disparities);
}


/*Normalised sum of squared intensity difference*/
void Naive::NaiveMatching_NormalisedSSD() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*for each horizontal disparity*/
			int minSSD = INT_MAX;
			int disparity = 0;

			for (int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d) {
				/*window around the epipolar line for both the images*/
				int nssd = 0;
				int den1, den2, mean1, mean2;
				int den = 1;
				den1 = den2 = mean1 = mean2 = 0;
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						/*calculate mean of each patch for both the images*/
						mean1 += this->img1.at<uchar>(r + i, c + j);
						mean1 /= (this->winSize * this->winSize);
						mean2 += this->img2.at<uchar>(r + i, c + j + d);
						mean2 /= (this->winSize * this->winSize);
						den1 = (this->img1.at<uchar>(r + i, c + j) - mean1) * (this->img1.at<uchar>(r + i, c + j) - mean1);
						den2 = (this->img2.at<uchar>(r + i, c + j + d) - mean2) * (this->img2.at<uchar>(r + i, c + j + d) - mean2);
						den += den1 + den2;
						den = std::sqrt(den) / 2;
						nssd += std::pow((this->img1.at<uchar>(r + i, c + j) - mean1 - this->img2.at<uchar>(r + i, c + j + d) + mean2), 2);
					}
				}
				if (den != 0)
					nssd /= den;
				else
					nssd;
				if (nssd < minSSD) {
					minSSD = nssd;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	std::string fileName = "NaiveMatching_NSSD.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

/*https://core.ac.uk/download/pdf/51249268.pdf*/
/*standard cross correlation*/
void Naive::NaiveMatching_CrossCorrelation() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*for each horizontal disparity*/
			int maxCC = INT_MIN;
			int disparity = 0.;

			for (int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d) {
				/*window around the epipolar line for both the images*/
				int cc = 0;
				int mean1, mean2;
				mean1 = mean2 = 0;
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						/*calculate mean of each patch for both the images*/
						mean1 += this->img1.at<uchar>(r + i, c + j);
						mean1 /= (this->winSize * this->winSize);
						mean2 += this->img2.at<uchar>(r + i, c + j + d);
						mean2 /= (this->winSize * this->winSize);
						cc += (this->img1.at<uchar>(r + i, c + j) - mean1) * (this->img2.at<uchar>(r + i, c + j + d) - mean2);
					}
				}
				if (cc > maxCC) {
					maxCC = cc;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	std::string fileName = "NaiveMatching_CC.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

void Naive::NaiveMatching_NormalisedCrossCorrelation() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*for each pixel in the image*/
#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*for each horizontal disparity*/
			int maxNCC = INT_MIN;
			int disparity = 0.;

			for (int d = -c + this->winSize / 2; d < this->img1.cols - c - this->winSize / 2; ++d) {
				/*window around the epipolar line for both the images*/
				int den = 1;
				int ncc, den1, den2, mean1, mean2;
				ncc = den1 = den2 = mean1 = mean2 = 0;
				
				for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
					for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
						/*calculate mean of each patch for both the images*/
						mean1 += this->img1.at<uchar>(r + i, c + j);
						mean1 /= (this->winSize * this->winSize);
						mean2 += this->img2.at<uchar>(r + i, c + j + d);
						mean2 /= (this->winSize * this->winSize);
						den1 += (this->img1.at<uchar>(r + i, c + j) - mean1) * (this->img1.at<uchar>(r + i, c + j) - mean1);
						den2 += (this->img2.at<uchar>(r + i, c + j + d) - mean2) * (this->img2.at<uchar>(r + i, c + j + d) - mean2);
						den = std::sqrt(den1 * den2);
						ncc += ((this->img1.at<uchar>(r + i, c + j) - mean1) * (this->img2.at<uchar>(r + i, c + j + d) - mean2));
					}
				}
				if (den != 0)
					ncc /= den;
				else
					ncc;
				if (ncc > maxNCC) {
					maxNCC = ncc;
					disparity = abs(d);
				}
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, c - this->winSize / 2) = (disparity * 255) / dmin;
		}
	}
	std::string fileName = "NaiveMatching_NCC.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

void Naive::saveDisparityImage(std::string& fileName, cv::Mat& disparity) {
	cv::imwrite(fileName, disparity);
}

void Naive::NaiveMatching_OpenCV() {

}


