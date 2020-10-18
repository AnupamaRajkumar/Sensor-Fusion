#include "Filters.h"

using namespace std;
using namespace cv;


Filters::Filters() {
	this->kSize = 3;
	this->meanX = 0;
	this->meanY = 0;
	this->stdDev = 25;
}


void Filters::DenoiseImage(Mat& origImg, Mat& noiseImg){
	cv::Mat output;

	/*Box filter implementation*/
	output = this->BoxFilter(noiseImg);
	imshow("boxFiler", output);
	imwrite("boxFiler.png", output);
	cout << "-------------------------------------------------------------" << endl;
	cout << "Performance metrics of Box/Averaging filter" << endl;
	this->PerformanceMetrics(origImg, output);

	/*openCV implementation*/
	// gaussian
	cv::GaussianBlur(noiseImg, output, cv::Size(this->kSize, this->kSize), this->meanX, this->stdDev);
	cv::imshow("opencv_gaussian", output);
	imwrite("openCV_Gaussian.png", output);
	cout << "-------------------------------------------------------------" << endl;
	cout << "Performance metrics of openCV Gaussian filter" << endl;
	this->PerformanceMetrics(origImg, output);

	/*non-opencv implementation*/
	output = this->GaussFilter(noiseImg);
	imshow("Gaussian", output);
	imwrite("Gaussian.png", output);
	cout << "-------------------------------------------------------------" << endl;
	cout << "Performance metrics of written Gaussian filter" << endl;
	this->PerformanceMetrics(origImg, output);

	/*non-opencv implementation - separable kernel*/
	output = this->GaussFilterSeparable(noiseImg);
	imshow("GaussianSeparable", output);
	imwrite("Gaussian_Separable.png", output);
	cout << "----------------------------------------------------------------" << endl;
	cout << "Performance metrics of written Gaussian filter with separabale kernels" << endl;
	this->PerformanceMetrics(origImg, output);

	// median
	cv::medianBlur(noiseImg, output, this->kSize);
	cv::imshow("opencv_median", output);
	imwrite("openCV_median.png", output);
	cout << "-------------------------------------------------------------" << endl;
	cout << "Performance metrics of openCV Median filter" << endl;
	this->PerformanceMetrics(origImg, output);

	/*non-opencv implementation*/
	output = this->MedianFilter(noiseImg);
	imshow("Median", output);
	imwrite("Median.png", output);
	cout << "-------------------------------------------------------------" << endl;
	cout << "Performance metrics of written Median filter" << endl;
	this->PerformanceMetrics(origImg, output);

	// bilateral
	cv::bilateralFilter(noiseImg, output, kSize, 2 * kSize, kSize / 2);
	cv::imshow("opencv_bilateral", output);
	imwrite("openCV_bilateral.png", output);
	cout << "-------------------------------------------------------------" << endl;
	cout << "Performance metrics of openCV Bilateral filter" << endl;
	this->PerformanceMetrics(origImg, output);

}

/*Average or Box filter*/
Mat Filters::BoxFilter(Mat& img) {
	Mat filtImg = Mat::zeros(img.size(), img.type());
	/*ignoring border handling*/
	for (int r = kSize / 2; r < img.rows - kSize / 2; r++) {
		for (int c = kSize / 2; c < img.cols - kSize / 2; c++) {
			int sum = 0;
			for (int i = -kSize / 2; i <= kSize / 2; i++) {
				for (int j = -kSize / 2; j <= kSize / 2; j++) {
					sum += img.at<uchar>(r + i, c + j);
				}
			}
			filtImg.at<uchar>(r, c) = sum / (kSize * kSize);
		}
	}
	return filtImg.clone();
}
/*1D Gaussian Kernel*/
Mat Filters::GaussianKernel1D() {
	Mat kernel = Mat::ones(1, kSize, CV_32FC1);
	float k = 2.5;
	float rmax = sqrt(2 * pow(kSize / 2, 2));
	float sigma = rmax / k;
	float sum = 0.;
	for (int i = -kSize / 2; i <= kSize / 2; i++) {
		float denominator, power;
		denominator = sqrt(2 * CV_PI)*sigma;
		//denominator = 1;
		power = pow(((i) - this->meanX), 2) / (2 * pow(sigma, 2));
		kernel.at<float>(i + kSize / 2) = exp(-power) / denominator;
		sum = sum + kernel.at<float>(i + kSize / 2);
	}
	/*normalise the distribution*/
	kernel /= sum;
	return kernel;
}

/*2D Gaussian Kernel*/
Mat Filters::GaussianKernel2D() {
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
			xDist = pow(((r) - this->meanX), 2);
			yDist = pow(((c) - this->meanY), 2);
			power = 0.5*((xDist / pow(sigma, 2)) + (yDist / pow(sigma, 2)));
			kernel.at<float>(r + kSize / 2, c + kSize / 2) = exp(-power) / denominator;
			sum += kernel.at<float>(r + kSize / 2, c + kSize / 2);
		}
	}
	/*normalize the distribution*/
	kernel = kernel / sum;
	return kernel;
}

/*Gaussian filtering*/
Mat Filters::GaussFilter(Mat& img) {
	Mat filtImg = Mat::ones(img.size(), img.type());
	Mat kernel = this->GaussianKernel2D();
	/*ignoring border handling*/
	for (int r = kSize / 2; r < img.rows - kSize / 2; r++) {
		for (int c = kSize / 2; c < img.cols - kSize / 2; c++) {
			float sum = 0;
			for (int i = -kSize / 2; i <= kSize / 2; i++) {
				for (int j = -kSize / 2; j < kSize / 2; j++) {
					sum += img.at<uchar>(r + i, c + j) * kernel.at<float>(i + kSize / 2, j + kSize / 2);
				}
			}
			filtImg.at<uchar>(r, c) = sum;
		}
	}
	return filtImg.clone();
}

Mat Filters::GaussFilterSeparable(Mat& img) {
	//Mat filtImg = Mat::ones(img.size(), img.type());
	Mat filtImg = img.clone();
	Mat temp = filtImg.clone();
	Mat firstConv = filtImg.clone();
	Mat secondConv = filtImg.clone();
	/*first convolution*/
	Mat kernel1 = this->GaussianKernel1D();
	for (int r = kSize / 2; r < img.rows - kSize / 2; r++) {
		for (int c = kSize / 2; c < img.cols - kSize / 2; c++) {
			float sum = 0;
			for (int i = 0; i < kernel1.rows; i++) {
				for (int j = -kSize / 2; j < kSize / 2; j++) {
					sum += img.at<uchar>(r + i, c + j) * kernel1.at<float>(i, j + kSize / 2);
				}
			}
			firstConv.at<uchar>(r, c) = sum;
		}
	}
	transpose(firstConv, temp);
	/*second convolution*/
	for (int r = kSize / 2; r < temp.rows - kSize / 2; r++) {
		for (int c = kSize / 2; c < temp.cols - kSize / 2; c++) {
			int sum = 0;
			for (int i = 0; i < kernel1.rows; i++) {
				for (int j = -kSize / 2; j < kSize / 2; j++) {
					sum += temp.at<uchar>(r + i, c + j) * kernel1.at<float>(i, j + kSize / 2);
				}
			}
			secondConv.at<uchar>(r, c) = sum;
		}
	}
	/*transpose the result*/
	transpose(secondConv, filtImg);

	return filtImg.clone();
}

/*Median Filtering*/
Mat Filters::MedianFilter(Mat& img) {
	Mat filtImg = Mat::zeros(img.size(), img.type());
	/*ignoring border handling*/
	for (int r = kSize / 2; r < img.rows - kSize / 2; r++) {
		for (int c = kSize / 2; c < img.cols - kSize / 2; c++) {
			vector<uchar> val;
			for (int i = -kSize / 2; i <= kSize / 2; i++) {
				for (int j = -kSize / 2; j <= kSize / 2; j++) {
					val.push_back(img.at<uchar>(r + i, c + j));
				}
			}
			sort(val.begin(), val.end());
			uchar value = val[val.size() % 2];
			filtImg.at<uchar>(r, c) = value;
		}		
	}
	return filtImg.clone();
}

/*Calculating the performance metrics*/
void Filters::PerformanceMetrics(Mat& origImg, Mat& denoisedImg) {
	Mat diff = denoisedImg - origImg;
	/*Sum of square differences*/
	float ssD = 0.;
	for (int r = 0; r < origImg.rows; r++) {
		for (int c = 0; c < origImg.cols; c++) {
			ssD += pow(diff.at<uchar>(r, c), 2);
		}
	}
	ssD /= (origImg.rows * origImg.cols);
	cout << "Sum of squared difference of the image:" << ssD << endl;

	/*Mean absolute error*/
	float meanAbsDiff = 0.;
	for (int r = 0; r < origImg.rows; r++) {
		for (int c = 0; c < origImg.cols; c++) {
			meanAbsDiff += diff.at<uchar>(r, c);
		}
	}
	meanAbsDiff /= (origImg.rows * origImg.cols);
	cout << "Mean absolute error of the image:" << meanAbsDiff << endl;

	/*Root mean square error*/
	float meanSqrDiff = cv::mean(diff.mul(diff))[0];
	float rmsDiff = sqrt(meanSqrDiff);
	cout << "Root mean square error of the image:" << rmsDiff << endl;

	/*Peak Signal to Noise ratio*/
	float PSNR = 10.0f * std::log10(255 * 255 / meanSqrDiff);
	cout << "PSNR of the image : " << PSNR << endl;

	/*Structural similarity index map*/
	float ssimVal = this->SSIMCalculation(origImg, denoisedImg);
	cout << "Structural similarity of the image : " << ssimVal << endl;
}

/*http://www.cns.nyu.edu/pub/eero/wang03-reprint.pdf*/
/*Calculating structural similarity index*/
float Filters::SSIMCalculation(Mat& origImg, Mat& denoisedImg) {
	/*step 1 : Calculate mean intensity of each image*/
	float C1 = 6.5025, C2 = 58.5225;
	float mX = 0.;
	for (int r = 0; r < origImg.rows; r++) {
		for (int c = 0; c < origImg.cols; c++) {
			mX += origImg.at<uchar>(r, c);
		}
	}
	mX /= (origImg.rows*origImg.cols);

	float mY = 0.;
	for (int r = 0; r < denoisedImg.rows; r++) {
		for (int c = 0; c < denoisedImg.cols; c++) {
			mY += denoisedImg.at<uchar>(r, c);
		}
	}
	mY /= (denoisedImg.rows*denoisedImg.cols);

	/*step 2 : Calculate standard deviation of each image*/
	float sDevX = 0.;
	for (int r = 0; r < origImg.rows; r++) {
		for (int c = 0; c < origImg.cols; c++) {
			sDevX += pow((origImg.at<uchar>(r, c) - mX), 2);
		}
	}
	sDevX = sqrt(sDevX / (origImg.rows*origImg.cols));

	float sDevY = 0.;
	for (int r = 0; r < denoisedImg.rows; r++) {
		for (int c = 0; c < denoisedImg.cols; c++) {
			sDevY += pow((denoisedImg.at<uchar>(r, c) - mY), 2);
		}
	}
	sDevY = sqrt(sDevY / (denoisedImg.rows*denoisedImg.cols));

	/*step 3 : calculating the SSIM*/
	float ssim = 0.;
	float num = 0.;
	float den = 0.;
	num = (2 * mX * mY + C1)*(2 * sDevX * sDevY + C2);
	den = (mX * mX + mY * mY + C1)*(sDevX * sDevX + sDevY * sDevY + C2);
	ssim = num / den;
	return ssim;
}