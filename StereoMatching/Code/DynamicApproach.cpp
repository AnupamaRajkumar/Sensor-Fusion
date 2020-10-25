#include "DynamicApproach.h"

/******************************************************
No aggregation step
Problem formulated as an energy minimisation
*******************************************************/

Dynamic::Dynamic(cv::Mat& image1, cv::Mat& image2, int dmin, int window_size, double weight) {
	this->occlusion = weight;
	this->winSize = window_size;
	this->dmin = dmin;
	this->img1 = image1.clone();
	this->img2 = image2.clone();
	this->C = cv::Mat::zeros(image1.cols, image1.cols, image1.type());
	this->M = cv::Mat::zeros(C.size(), CV_8UC1);
}

/*Dynmaic approach calculation menu*/
void Dynamic::DynamicApproachCalculation() {
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*going through each horizontal scanline one by one*/
//#pragma omp parallel for
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {
		std::vector<uchar> DSI = this->DisparitySpaceImage(r);
		this->FindMinimumCostPath(DSI);
		this->ReconstructOptimalPath(r, naive_disparities);
	}
	std::string fileName = "DynamicProgrammingEnergyMin.png";
	this->saveDisparityImage(fileName, naive_disparities);
}

std::vector<uchar> Dynamic::DisparitySpaceImage(int r) {
	//cv::Mat DSI = cv::Mat::zeros(1, this->img1.cols, CV_8UC1);
	std::vector<uchar> DSI;
	DSI.reserve(this->img1.cols);
	for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*window around the epipolar line for both the images*/
			int sad = 0;
			for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
				for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
					sad += abs((img1.at<uchar>(r + i, c + j) - img2.at<uchar>(r + i, c + j)));
				}
			}
		DSI.push_back(sad);
	}
	return DSI;
}

void Dynamic::FindMinimumCostPath(std::vector<uchar>& DSI) {

	for (int r = 0; r < DSI.size(); r++) {
		//right occlusion
		C.at<uchar>(r, 0) = DSI[r];
		M.at<uchar>(r, 0) = RightOcc;
		//left occlusion
		C.at<uchar>(0, r) = DSI[r];
		M.at<uchar>(0, r) = LeftOcc;
	}


	for (int r = 1; r < C.rows; r++) {
		for (int c = 1; c < C.cols; c++) {
			double min1, min2, min3, minVal;
			min1 = min2 = min3 = minVal = 0.;
			min1 = C.at<uchar>(r - 1, c - 1) +  DSI[c];				//match
			min2 = C.at<uchar>(r - 1, c) + this->occlusion;			//left occlusion
			min3 = C.at<uchar>(r, c - 1) + this->occlusion;			//right occlusion
			/*determine the min of 3*/
			minVal = min(min1, min(min2, min3));
			/*Populate C and M matrices accordingly*/
			C.at<uchar>(r, c) = minVal;
			if (minVal == min1)
				M.at<uchar>(r, c) = Match;
			if (minVal == min2)
				M.at<uchar>(r, c) = LeftOcc;
			if (minVal == min3)
				M.at<uchar>(r, c) = RightOcc;
		}
	}
	//cv::imwrite("CostMatrix.png", C);
}

/*http://www.epixea.com/research/multi-view-coding-thesisch3.html#ref-Bobick1999*/
void Dynamic::ReconstructOptimalPath(int r, cv::Mat& naive_disparities) {
	int row = M.rows - 1;
	int col = M.cols - 1;
	int disparity = 0;
	while (row != 0 && col != 0) {
		//std::cout << int(M.at<uchar>(row, col)) << std::endl;
		switch (int(M.at<uchar>(row, col))) {
		case Match:
			disparity = row;
			row--;
			col--;
			break;
		case LeftOcc:
			disparity = row - 1;
			row--;
			break;
		case RightOcc:
			//disparity = abs(row);			
			col--;
			break;
		}
		//std::cout << "disparity:" << disparity << std::endl;
		naive_disparities.at<uchar>(r - this->winSize / 2, col) = (disparity * 255) / dmin;				//;		
		//std::cout << int(naive_disparities.at<uchar>(r - this->winSize / 2, col)) << std::endl;
	}
}

void Dynamic::saveDisparityImage(std::string& fileName, cv::Mat& disparity) {
	cv::imwrite(fileName, disparity);
}
