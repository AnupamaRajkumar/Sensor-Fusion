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
	this->C = cv::Mat::zeros(image1.cols, image1.cols, CV_32FC1);
	this->M = cv::Mat::zeros(C.size(), CV_8UC1);
}

/*Dynmaic approach calculation menu*/
void Dynamic::DynamicApproachCalculation() {

	//this->calculateDynamicStereo();
	Utility utility;
	cv::Mat naive_disparities = cv::Mat::zeros(this->img1.size(), CV_8UC1);
	/*going through each horizontal scanline one by one*/
	for (int r = this->winSize / 2; r < this->img1.rows - this->winSize / 2; r++) {

		C.at<float>(0, 0) = this->occlusion;
		M.at<uchar>(0, 0) = RightOcc;
		for (int i = 1; i < C.rows; i++) {
			C.at<float>(i, 0) = C.at<float>(i - 1, 0) + this->occlusion;
			M.at<uchar>(i, 0) = LeftOcc;
		}
		for (int i = 1; i < C.cols; i++) {
			C.at<float>(0, i) = C.at<float>(0, i - 1) + this->occlusion;
			M.at<uchar>(0, i) = RightOcc;
		}
#pragma omp parallel for
		for (int lr = 1; lr < C.rows; lr++) {
			//std::cout << row << " " << r << std::endl;
			for (int rr = 1; rr < C.cols; rr++) {
				//std::cout << c << std::endl;
				float min1, min2, min3, minVal;
				min1 = min2 = min3 = minVal = 0.;
				float disp = this->DisparitySpaceImage(r, lr, rr);
				min1 = C.at<float>(lr - 1, rr - 1) + disp;		//match
				min2 = C.at<float>(lr - 1, rr) + this->occlusion;									//left occlusion
				min3 = C.at<float>(lr, rr - 1) + this->occlusion;									//right occlusion
				/*determine the min of 3*/
				minVal = min(min1, min(min2, min3));
				/*Populate C and M matrices accordingly*/
				float epsilon = 0.0000001f;
				C.at<float>(lr, rr) = minVal;
				if (fabs(minVal - min1) < epsilon) {
					M.at<uchar>(lr, rr) = Match;
				}
				if (fabs(minVal - min2) < epsilon) {
					M.at<uchar>(lr, rr) = LeftOcc;
				}
				if (fabs(minVal - min3) < epsilon) {
					M.at<uchar>(lr, rr) = RightOcc;
				}
			}
		}

		int row = M.rows - 1;
		int col = M.cols - 1;
		int disparity = 0;
		int iterator = naive_disparities.cols;
		while (row != 0 && col != 0) {
			switch (int(M.at<uchar>(row, col))) {
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
		//this->FindMinimumCostPath(r);
		//this->ReconstructOptimalPath(r, naive_disparities);
	}
	std::string fileName = "DynamicProgrammingEnergyMin.png";
	utility.saveDisparityImage(fileName, naive_disparities);
	std::string cloudFileSGBM = "DynamicProgrammingEnergyMin";
	utility.Disparity2PointCloud(cloudFileSGBM, naive_disparities, this->winSize, this->dmin, this->baseline,
								 this->focal_length, this->cx_d, this->cy_d, this->doffs);

}

float Dynamic::DisparitySpaceImage(int row, int lr, int rr) {
	float sad = 0;
	for (int c = this->winSize / 2; c < this->img1.cols - this->winSize / 2; c++) {
			/*window around the epipolar line for both the images*/		
			for (int i = -this->winSize / 2; i < this->winSize / 2; i++) {
				for (int j = -this->winSize / 2; j < this->winSize / 2; j++) {
					sad += abs(img1.at<uchar>(row + j, c + i) - img2.at<uchar>(row + j, c + i));
				}
			}
	}
	return sad;
}

void Dynamic::FindMinimumCostPath(int row) {

}

/*http://www.epixea.com/research/multi-view-coding-thesisch3.html#ref-Bobick1999*/
void Dynamic::ReconstructOptimalPath(int r, cv::Mat& naive_disparities) {

}

#if 0
bool imagesContaining(cv::Point source, cv::Point target, cv::Mat& img1, cv::Mat& img2) {
	bool test1 = (source.x >= 0 && source.x < img1.cols && source.y >= 0 && source.y < img1.rows);
	bool test2 = (target.x >= 0 && target.x < img2.cols && target.y >= 0 && target.y < img2.rows);

	return(test1 && test2);
}

double SSDCost(cv::Point left_point, int right_index, int window_size, cv::Mat& img1, cv::Mat& img2) {
	double error = 0;

	int half_window_size = window_size / 2;
	for (int k = -half_window_size; k <= half_window_size; k++) {
		for (int j = -half_window_size; j <= half_window_size; j++) {

			if (imagesContaining(cv::Point(left_point.x + k, left_point.y + j), cv::Point(right_index + k, left_point.y + j), img1, img2)) {
				error += pow(img1.at<uchar>(left_point.y + j, left_point.x + k) - img2.at<uchar>(left_point.y + j, right_index + k), 2);
			}

		}
	}

	return error;
}
double calculateDissim(cv::Point left, cv::Point right, int window_size, cv::Mat& img1, cv::Mat& img2) {
	double dissim = 0;
	dissim = SSDCost(left, right.x, window_size, img1, img2);
	return dissim;
}



void Dynamic::calculateDynamicStereo() {

	int rowsToProcess = this->img1.rows;
	int progress = 0;
	int barWidth = 50;
	auto start_time = std::chrono::high_resolution_clock::now();

	std::cout << "Starting Dynamic Programming stereo calculation with lambda = " << this->occlusion << std::endl;

	cv::Mat dispImage = cv::Mat(this->img1.rows, this->img2.cols, CV_8UC1, cv::Scalar(0));
	
#pragma omp parallel for
	for (int row = 0; row < this->img1.rows; row++) {
		cv::Mat left_scan = this->img1.row(row);
		cv::Mat right_scan = this->img2.row(row);

		cv::Mat dissimilarity = cv::Mat(left_scan.cols, right_scan.cols, CV_64F, cv::Scalar(0));
		cv::Mat occlusionImage = cv::Mat(left_scan.cols, right_scan.cols, CV_8U, cv::Scalar(0));

		//Initialize the edges if the dissimilarity image
		dissimilarity.at<double>(0, 0) = this->occlusion;
		occlusionImage.at<uchar>(0, 0) = RightOcc;
		for (int i = 1; i < dissimilarity.rows; i++) {
			dissimilarity.at<double>(i, 0) = dissimilarity.at<double>(i - 1, 0) + this->occlusion;
			occlusionImage.at<uchar>(i, 0) = LeftOcc;
		}
		for (int j = 1; j < dissimilarity.cols; j++) {
			dissimilarity.at<double>(0, j) = dissimilarity.at<double>(0, j - 1) + this->occlusion;
			occlusionImage.at<uchar>(0, j) = RightOcc;
		}
		double mincost = 10e10;
		// Calculate the inner min dissimilarities for the matrix
		for (int r = 1; r < left_scan.cols; r++) {
			for (int c = 1; c < right_scan.cols; c++) {

				double match_cost = dissimilarity.at<double>(r - 1, c - 1) + calculateDissim(cv::Point(r, row), cv::Point(c, row), this->winSize, this->img1, this->img2);
				double left_occlusion = dissimilarity.at<double>(r - 1, c) + this->occlusion;
				double right_occlusion = dissimilarity.at<double>(r, c - 1) + this->occlusion;

				if (match_cost < min(left_occlusion, right_occlusion)) {
					dissimilarity.at<double>(r, c) = match_cost;
					occlusionImage.at<uchar>(r, c) = Match;

					//dispImage.at<uchar>(row, r) = r - c; //Correspondences are always at biger pixel number for left image than right image
				}
				else if (left_occlusion < min(match_cost, right_occlusion)) {
					dissimilarity.at<double>(r, c) = left_occlusion;
					occlusionImage.at<uchar>(r, c) = LeftOcc;
				}
				else {
					dissimilarity.at<double>(r, c) = right_occlusion;
					occlusionImage.at<uchar>(r, c) = RightOcc;
				}

				if (match_cost < mincost) {
					mincost = match_cost;
				}

			}
		}

		//If the dissim matrix is calculated, we need to find the optimal path
		//We always have to note left scan corresponds for rows and right scan is for cols
		//Pixel values for left scan is always bigger -> o_row > o_col
		int o_row = occlusionImage.rows - 1;
		int o_col = occlusionImage.cols - 1;
		int disparity = 0;
		int iterator = dispImage.cols;
		while (o_row != 0 && o_col != 0) {
			int occlusion_value = occlusionImage.at<uchar>(o_row, o_col);
			switch (occlusion_value) {
			case Match:
				disparity = abs(o_row - o_col);
				iterator--;
				o_row--;
				o_col--;
				break;
			case LeftOcc:
				o_row--;
				break;
			case RightOcc:
				o_col--;
				break;
			}
			dispImage.at<uchar>(row, iterator) = (disparity * 255) / 127;

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

	cv::imwrite("DynamicProgramming.png", dispImage);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start_time);
	std::cout << "Dynamic Stereo Calculation took " << duration.count() << "s to process" << std::endl;

}

#endif

