#include "DynamicApproach.h"
#include "Utility.h"

/******************************************************
No aggregation step
Problem formulated as an energy minimisation
*******************************************************/

Dynamic::Dynamic(cv::Mat& image1, cv::Mat& image2, int window_size, float weight, int dmin,
				 double focal_length, double baseline, double cx_d, double cy_d, double doffs) {
	this->occlusion = weight;
	this->winSize = window_size;
	this->dmin = dmin;
	this->cx_d = cx_d;
	this->cy_d = cy_d;
	this->focal_length = focal_length;
	this->baseline = baseline;
	this->doffs = doffs;
	this->img1 = image1.clone();
	this->img2 = image2.clone();
}

/*Dynmaic approach calculation menu*/
void Dynamic::DynamicApproachCalculation() {

	int rowsToProcess = this->img1.rows;
	int progress = 0;
	int barWidth = 50;
	Utility utility;
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
#pragma omp parallel for
	/*going through each horizontal scanline one by one*/
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		cv::Mat C = cv::Mat::zeros(this->img1.cols, this->img2.cols, CV_32F);
		cv::Mat M = cv::Mat::zeros(C.size(), CV_8U);

		C.at<float>(0, 0) = this->occlusion;
		M.at<uchar>(0, 0) = RightOcc;
		for (int i = 1; i < C.rows; i++) {
			C.at<float>(i, 0) = C.at<float>(i - 1, 0) + this->occlusion;
			M.at<uchar>(i, 0) = LeftOcc;
		}
		for (int j = 1; j < C.cols; j++) {
			C.at<float>(0, j) = C.at<float>(0, j - 1) + this->occlusion;
			M.at<uchar>(0, j) = RightOcc;
		}
		for (int lr = 1; lr < C.rows; lr++) {
			for (int rr = 1; rr < C.cols; rr++) {
				float min1 = C.at<float>(lr - 1, rr - 1) + this->DisparitySpaceImage(r, lr, rr);		//match
				float min2 = C.at<float>(lr - 1, rr) + this->occlusion;									//left occlusion
				float min3 = C.at<float>(lr, rr - 1) + this->occlusion;									//right occlusion
				/*Populate C and M matrices depending on the minimum of the computed values*/
				if (min1 < min(min2, min3)) {
					C.at<float>(lr, rr) = min1;
					M.at<uchar>(lr, rr) = Match;
				}
				else if (min2 < min(min1, min3)) {
					C.at<float>(lr, rr) = min2;
					M.at<uchar>(lr, rr) = LeftOcc;
				}
				else {
					C.at<float>(lr, rr) = min3;
					M.at<uchar>(lr, rr) = RightOcc;
				}
			}
		}
		/*http://www.epixea.com/research/multi-view-coding-thesisch3.html#ref-Bobick1999*/
		int row = M.rows - 1;
		int col = M.cols - 1;
		int disparity = 0;
		int iterator = naive_disparities.cols;
		while (row != 0 && col != 0) {
			int occVal = M.at<uchar>(row, col);
			switch (occVal) {
			case Match:
				disparity = abs(row - col);
				row--;
				col--;
				iterator--;
				break;
			case LeftOcc:
				row--;
				break;
			case RightOcc:
				col--;
				break;
			}
			naive_disparities.at<uchar>(r - this->winSize / 2, iterator) = (disparity * 255) / this->dmin;
		}

		std::cout << "[";
		int pos = barWidth * ((progress) / static_cast<double>(rowsToProcess));
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "=";
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int((progress) / static_cast<double>(rowsToProcess) * 100.0) << " %\r";
		std::cout.flush();
		progress++;
	}
	std::string fileName = "DynamicProgrammingEnergyMin.png";
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudDP = "DynamicProgrammingEnergyMin";
	//utility.Disparity2PointCloud(cloudDP, naive_disparities, this->winSize, this->dmin, this->baseline,
	//							 this->focal_length, this->cx_d, this->cy_d, this->doffs);

}

float Dynamic::DisparitySpaceImage(int row, int lr, int rr) {
	float ssd = 0.;
	/*window around the epipolar line for both the images*/	
	for (int i = -this->winSize / 2; i <= this->winSize / 2; i++) {
		for (int j = -this->winSize / 2; j <= this->winSize / 2; j++) {
			ssd += pow((img1.at<uchar>(row + j, lr + i) - img2.at<uchar>(row + j, rr + i)), 2);
		}
	}
	return ssd;
}


