#include "Bilateral.h"

Bilateral::Bilateral() {
	this->kSize = 5;
	this->sigma_radiometric = 2 * kSize;
	this->sigma_spatial = kSize / 2;
}

void Bilateral::BilateralFilteringMenu(Mat& noisyImg, Mat& imFlash, Mat& imNoFlash) {
	Mat output;

	/*OpenCV Bilateral filter*/
	output = this->OpenCVBilateral(noisyImg);
	cv::imshow("OpenCVbilateral", output);
	imwrite("OpenCV_Bilateral.png", output);

	/*Written bilateral filtering - Method 1*/
	output = this->BilateralImplementation_1(noisyImg);
	imshow("BilateralFiltering_Method1", output);
	imwrite("BilateralFilter_Method1.png", output);

	/*Written bilateral filtering - Method 2*/
	output = this->BilateralImplementation_2(noisyImg);
	imshow("BilateralFilter_Method2", output);
	imwrite("BilateralFilter_Method2.png", output);

	/*Joint bilateral filtering*/
	output = this->JointBilateralUpsampling_1(imFlash, imNoFlash);
	imshow("JointBilateralFiltering_Method1", output);
	imwrite("JointBilateralFiltering_Method1.png", output);

	/*Joint bilateral filtering*/
	output = this->JointBilateralUpsampling_2(imFlash, imNoFlash);
	imshow("JointBilateralFiltering_Method2", output);
	imwrite("JointBilateralFiltering_Method2.png", output);
}

Mat Bilateral::OpenCVBilateral(Mat& noisyImg) {
	Mat filtImg = Mat::ones(noisyImg.size(), noisyImg.type());
	bilateralFilter(noisyImg, filtImg, kSize, 2 * kSize, kSize / 2);
	return filtImg.clone();
}

/*2D Gaussian Kernel*/
Mat Bilateral::GaussianKernel2D() {
	Mat kernel = Mat::ones(kSize, kSize, CV_32FC1);
	float k = 2.5;
	float rmax = sqrt(2 * pow(kSize / 2, 2));
	float sigma = rmax / k;
	float sum = 0.0;
	for (int r = -kSize / 2; r <= kSize / 2; r++) {
		for (int c = -kSize / 2; c <= kSize / 2; c++) {
			float denominator, power, xDist, yDist;
			denominator = 2 * CV_PI * sigma * sigma;
			//denominator = 1;
			xDist = pow(r, 2);
			yDist = pow(c, 2);
			power = 0.5*((xDist / pow(sigma, 2)) + (yDist / pow(sigma, 2)));
			kernel.at<float>(r + kSize / 2, c + kSize / 2) = exp(-power) / denominator;
			sum += kernel.at<float>(r + kSize / 2, c + kSize / 2);
		}
	}
	/*normalize the distribution*/
	kernel = kernel / sum;
	return kernel;
}

/*https://en.wikipedia.org/wiki/Gaussian_filter*/

Mat Bilateral::BilateralImplementation_1(Mat& noisyImg) {
	Mat filtImg = Mat::ones(noisyImg.size(), noisyImg.type());
	Mat kernel = this->GaussianKernel2D();
	int half = this->kSize / 2;

	for (int r = half; r < noisyImg.rows - half; r++) {
		for (int c = half; c < noisyImg.cols - half; c++) {
			float sum = 0;
			float wtSum = 0;
			for (int i = -half; i <= half; i++) {
				for (int j = -half; j <= -half; j++) {
					float diff = abs(noisyImg.at<uchar>(r, c) - noisyImg.at<uchar>(r + i, c + j));
					float weight = this->p(diff) * kernel.at<float>(i + half, j + half);
					sum += noisyImg.at<uchar>(r + i, c + j) * weight;
					wtSum += weight;
				}
			}
			filtImg.at<uchar>(r, c) = sum / wtSum;
		}
	}
	return filtImg.clone();
}

float Bilateral::p(float diff) {
	float pVal = 0;
	int sigma = kSize;
	float num = exp(-(diff) / (2 * sigma * sigma));
	float den = sqrt(2 * CV_PI) * sigma;
	pVal = num / den;
	return pVal;
}

float Bilateral::CalculateKernelBilateralFilter(float distance, float sigma)
{
	return exp(-(pow(distance, 2)) / (2 * pow(sigma, 2))) / (2 * CV_PI * pow(sigma, 2));
}

float Bilateral::distance(int currentX, int currentY, int neighborX, int neighborY)
{
	return float(sqrt(pow(currentX - neighborX, 2) + pow(currentY - neighborY, 2)));
}

/*https://en.wikipedia.org/wiki/Bilateral_filter*/

Mat Bilateral::BilateralImplementation_2(Mat& noisyImg) {
	Mat filtImg = Mat::ones(noisyImg.size(), noisyImg.type());
	float imgFilt = 0, accWt = 0;
	float hSpat = 1, hRad = 1, wtCombined = 1;
	int neighbourX = 0, neighbourY = 0;
	int half = this->kSize / 2;

	for (int i = half; i < noisyImg.rows - half; i++) {
		for (int j = half; j < noisyImg.cols - half; j++) {
			for (int row = 0; row < this->kSize; row++)
			{
				for (int col = 0; col < this->kSize; col++)
				{
					neighbourX = i - half + row;
					neighbourY = j - half + col;
					if (not(neighbourX < 0 or neighbourX > noisyImg.rows or neighbourY < 0 or neighbourY > noisyImg.cols)) {
						hSpat = CalculateKernelBilateralFilter(distance(i, j, neighbourX, neighbourY), sigma_spatial);
						/*calculate range filtering component from the high quality flash image to preserve edges*/
						hRad = CalculateKernelBilateralFilter(noisyImg.at<uchar>(neighbourX, neighbourY) - noisyImg.at<uchar>(i, j), sigma_radiometric);
						wtCombined = hSpat * hRad;
						imgFilt = imgFilt + (noisyImg.at<uchar>(neighbourX, neighbourY) * wtCombined);
						accWt = accWt + wtCombined;
					}
				}
			}
			imgFilt = imgFilt / accWt;
			filtImg.at<uchar>(i , j) = imgFilt;
			accWt = 0;
			imgFilt = 0;
		}
	}
	return filtImg.clone();
}

Mat Bilateral::JointBilateralUpsampling_1(Mat& imFlash, Mat& imNoFlash) {
	Mat filtImg = Mat::ones(imFlash.size(), imFlash.type());
	Mat kernel = this->GaussianKernel2D();
	int half = this->kSize / 2;

	for (int r = half; r < imFlash.rows - half; r++) {
		for (int c = half; c < imFlash.cols - half; c++) {
			float sum = 0;
			float wtSum = 0;
			for (int i = -half; i <= half; i++) {
				for (int j = -half; j <= -half; j++) {
					float diff = abs(imFlash.at<uchar>(r, c) - imFlash.at<uchar>(r + i, c + j));
					float weight = this->p(diff) * kernel.at<float>(i + half, j + half);
					sum += imNoFlash.at<uchar>(r + i, c + j) * weight;
					wtSum += weight;
				}
			}
			filtImg.at<uchar>(r, c) = sum / wtSum;
		}
	}
	return filtImg.clone();
}

/*https://johanneskopf.de/publications/jbu/paper/FinalPaper_0185.pdf*/
Mat Bilateral::JointBilateralUpsampling_2(Mat& imFlash, Mat& imNoFlash) {
	Mat filtImg = Mat::ones(imNoFlash.size(), imNoFlash.type());
	float imgFilt = 0.;
	float accWt = 0.;
	float hSpat = 1., hRad = 1.;
	float wtCombined = 1.;
	int neighbourX = 0, neighbourY = 0;
	int half = this->kSize / 2;

	for (int i = half; i < imNoFlash.rows - half; i++) {
		for (int j = half; j < imNoFlash.cols - half; j++) {
			for (int row = 0; row < this->kSize; row++)
			{
				for (int col = 0; col < this->kSize; col++)
				{
					neighbourX = i - half + row;
					neighbourY = j - half + col;
					
					if (not(neighbourX < 0 or neighbourX > imNoFlash.rows or neighbourY < 0 or neighbourY > imNoFlash.cols)) {
						hSpat = CalculateKernelBilateralFilter(distance(i, j, neighbourX, neighbourY), sigma_spatial);
						hRad = CalculateKernelBilateralFilter(imFlash.at<uchar>(neighbourX, neighbourY) - imFlash.at<uchar>(i, j), sigma_radiometric);
						wtCombined = hSpat * hRad;
						imgFilt = imgFilt + (imNoFlash.at<uchar>(neighbourX, neighbourY) * wtCombined);
						accWt = accWt + wtCombined;
					}
				}
			}
			imgFilt = imgFilt / accWt;
			filtImg.at<uchar>(i, j) = imgFilt;
			accWt = 0;
			imgFilt = 0;

		}
	}
	return filtImg.clone();
}

/*To be done*/
Mat Bilateral::BilateralMedianFiltering(Mat& noisyImg) {
	Mat filtImg = Mat::ones(noisyImg.size(), noisyImg.type());

	return filtImg.clone();
}

