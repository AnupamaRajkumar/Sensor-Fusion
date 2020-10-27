#include "NaiveApproach.h"
#include "Utility.h"

/*****************************************************************************
Naive approach primarily consists of following steps ie. a typical stereo pipeline
comprises of following steps:
1. Matching cost computation
2. Cost aggregation
3. Disparity computation
4. Disparity refinement - consistency check, hole filling, removing spurious matches etc
**********************************************************************************/

Utility utility;

/*Constructor*/
Naive::Naive(int window_size, cv::Mat& image1, cv::Mat& image2, int dmin, 
			 double focal_length, double baseline, double cx_d, double cy_d, double doffs) {
	this->winSize = window_size;	
	this->img1 = image1.clone();
	this->img2 = image2.clone();
	this->dmin = dmin;
	this->dmax = 128;
	this->focal_length = focal_length;
	this->baseline = baseline;
	this->cx_d = cx_d;
	this->cy_d = cy_d;
	this->doffs = doffs;
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::cout << "Saving point cloud" << std::endl;
	std::string cloudFile = "NaiveMatching_SAD";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline, 
								 this->focal_length, this->cx_d, this->cy_d, this->doffs);
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFile = "NaiveMatching_ZSAD";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFile = "NaiveMatching_LSSAD";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFile = "NaiveMatching_SSD";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFile = "NaiveMatching_NSSD";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFile = "NaiveMatching_CC";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
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
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFile = "NaiveMatching_NCC";
	utility.Disparity2PointCloud(cloudFile, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
}

void Naive::NaiveMatching_OpenCV() {

	/*stereoBM opencv implementation*/
	std::cout << "-------------OpenCV StereoBM---------------" << std::endl;
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_16S);
	cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(this->dmax, this->winSize);
	sbm->compute(this->img1, this->img2, naive_disparities);
	std::string fileNameBM = "OpenCV_StereoBM.png";
	utility.saveDisparityImage(fileNameBM, naive_disparities);
	std::string cloudFileBM = "OpenCV_StereoBM";
	utility.Disparity2PointCloud(cloudFileBM, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
	std::cout << "------------StereoBM generated-------------" << std::endl;

	/*StereoBinarySGBM*/
	std::cout << "------------OpenCV StereoSGBM---------------" << std::endl;
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, this->dmax, this->winSize,
														 (8 * this->winSize*this->winSize), 
														 (32 * this->winSize*this->winSize), 
														  1, 0, 5, 64, 16, cv::StereoSGBM::MODE_SGBM);
	sgbm->compute(this->img1, this->img2, naive_disparities);
	std::string fileNameSGBM = "OpenCV_StereoSGBM.png";
	utility.saveDisparityImage(fileNameSGBM, naive_disparities);
	std::string cloudFileSGBM = "OpenCV_StereoSGBM";
	utility.Disparity2PointCloud(cloudFileSGBM, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
	std::cout << "------------StereoSGBM generated-------------" << std::endl;

	/*StereoBinaryHH*/
	std::cout << "-------------OpenCV StereoHH------------------" << std::endl;
	cv::Ptr<cv::StereoSGBM> hh = cv::StereoSGBM::create(0, this->dmax, this->winSize,
														 (8 * this->winSize*this->winSize),
														 (32 * this->winSize*this->winSize),
														  1, 0, 5, 64, 16, cv::StereoSGBM::MODE_HH);
	hh->compute(this->img1, this->img2, naive_disparities);
	std::string fileNameHH = "OpenCV_StereoHH.png";
	utility.saveDisparityImage(fileNameHH, naive_disparities);
	std::string cloudFileHH = "OpenCV_StereoSGBM";
	utility.Disparity2PointCloud(cloudFileHH, naive_disparities, this->winSize, this->dmin, this->baseline,
		this->focal_length, this->cx_d, this->cy_d, this->doffs);
	std::cout << "--------------StereoHH generated----------------" << std::endl;
}
	


