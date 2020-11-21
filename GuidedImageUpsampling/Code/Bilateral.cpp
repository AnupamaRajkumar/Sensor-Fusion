#include "Bilateral.h"
#include "Utility.h"

Bilateral::Bilateral(Mat& origImg, Mat& noisyImg, Mat& guidedImg, Mat& downImg, int kSize) {
	this->kSize = kSize;
	this->origImg = origImg;
	this->noisyImg = noisyImg;
	this->guidedImg = guidedImg;
	this->downImg = downImg;
}

void Bilateral::BilateralFilteringMenu() {
	int choice = 1;
	cout << "Bilateral Filtering Menu" << endl;
	cout << "1. Bilateral Filtering" << endl;
	cout << "2. Bilateral Median Filtering" << endl;
	cout << "Enter your choice (1/2)" << endl;
	cin >> choice;
	switch (choice) {
		case 1:
			this->CalculateBilateralFiltering();
			break;
		case 2:
			this->CalculateBilateralMedianFiltering();
			break;
		default:
			cout << "Kindly enter valid choice" << endl;
			break;
	}
}

void Bilateral::CalculateBilateralFiltering() {
	/*For bilateral filtering, we try out output for different parameters*/
	float sigmaRadVals[4] = { this->kSize, 2 * this->kSize, 3 * this->kSize, 4 * this->kSize };
	float sigmaSpatVals[4] = { this->kSize / 4, this->kSize / 3, this->kSize / 2, this->kSize };
	for (int i = 0; i < 4; i++) {			//sigmaSpat
		for (int j = 0; j < 4; j++) {		//sigmaRad
			Mat filtImg;
			cout << "Bilateral filtering: Image number" << i + 1 << endl;
			cout << "Spatial std dev:" << sigmaSpatVals[i] << ",Radial std dev:" << sigmaRadVals[j] << endl;
			filtImg = this->BilateralImplementation(sigmaSpatVals[i], sigmaRadVals[j]);
			string fileName1 = to_string(sigmaSpatVals[i]);
			string fileName2 = to_string(sigmaRadVals[j]);
			string fileName = "BilateralFiltering_" + fileName1 + "_" + fileName2 + ".png";
			imwrite(fileName, filtImg);
		}
	}
}

void Bilateral::CalculateBilateralMedianFiltering() {
	/*For bilateral filtering, we try out output for different parameters*/
	float sigmaRadVals[4] = { this->kSize, 2 * this->kSize, 3 * this->kSize, 4 * this->kSize };
	float sigmaSpatVals[4] = { this->kSize/4, this->kSize/3, this->kSize/2, this->kSize };
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			Mat filtImg;
			cout << "Bilateral median filtering: Image number" << i + 1 << endl;
			cout << "Spatial std dev:" << sigmaSpatVals[i] << ",Radial std dev:" << sigmaRadVals[j] << endl;
			filtImg = this->BilateralMedianFiltering(sigmaSpatVals[i], sigmaRadVals[j]);
			string fileName1 = to_string(sigmaSpatVals[i]);
			string fileName2 = to_string(sigmaRadVals[j]);
			string fileName = "BilateralMedianFiltering_" + fileName1 + "_" + fileName2 + ".png";
			imwrite(fileName, filtImg);
		}
	}
}

float Bilateral::CalculateKernelBilateralFilter(float distance, float sigma)
{
	return exp(-(pow(distance, 2)) / (2 * pow(sigma, 2))) / (2 * CV_PI * pow(sigma, 2));
}

float Bilateral::distance(int currentX, int currentY, int neighborX, int neighborY)
{
	return float(sqrt(pow(currentX - neighborX, 2) + pow(currentY - neighborY, 2)));
}

/* https://en.wikipedia.org/wiki/Bilateral_filter */

Mat Bilateral::BilateralImplementation(float sigmaSpat, float sigmaRad) {
	Mat filtImg = Mat::ones(noisyImg.size(), noisyImg.type());
	float imgFilt = 0, accWt = 0;
	float hSpat = 1, hRad = 1, wtCombined = 1;
	int neighbourX = 0, neighbourY = 0;
	int half = this->kSize / 2;

	for (int i = 0; i < noisyImg.rows; i++) {
		for (int j = 0; j < noisyImg.cols; j++) {
			for (int row = 0; row < this->kSize; row++)
			{
				for (int col = 0; col < this->kSize; col++)
				{
					neighbourX = i + row;
					neighbourY = j + col;
					if (not(neighbourX < 0 or neighbourX > noisyImg.rows or neighbourY < 0 or neighbourY > noisyImg.cols)) {
						/*calculate the spatial filtering component*/
						hSpat = CalculateKernelBilateralFilter(distance(i, j, neighbourX, neighbourY), sigmaSpat);
						/*calculate range filtering component */
						hRad = CalculateKernelBilateralFilter(noisyImg.at<uchar>(neighbourX, neighbourY) - noisyImg.at<uchar>(i, j), sigmaRad);
						wtCombined = hSpat * hRad;
						imgFilt = imgFilt + (noisyImg.at<uchar>(neighbourX, neighbourY) * wtCombined);
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

/* https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.385.5803&rep=rep1&type=pdf */

Mat Bilateral::BilateralMedianFiltering(float sigmaSpat, float sigmaRad) {
	Mat filtImg = Mat::ones(noisyImg.size(), noisyImg.type());
	float hSpat = 1, hRad = 1, wtCombined = 1;
	int neighbourX = 0, neighbourY = 0;
	int half = this->kSize / 2;

	for (int i = 0; i < noisyImg.rows; i++) {
		for (int j = 0; j < noisyImg.cols; j++) {
			vector<pair<uchar, float>> imgFiltVec;
			float accWt = 0., filtVal = 0.;
			uchar imgVal = 0;
			for (int row = 0; row < this->kSize; row++)
			{
				for (int col = 0; col < this->kSize; col++)
				{
					neighbourX = i + row;
					neighbourY = j + col;
					if (not(neighbourX < 0 or neighbourX > noisyImg.rows or neighbourY < 0 or neighbourY > noisyImg.cols))
					{
						pair<uchar, float> imgFilt;
						/* calculate the spatial filtering component*/
						hSpat = CalculateKernelBilateralFilter(distance(i, j, neighbourX, neighbourY), sigmaSpat);
						/* calculate range filtering component */
						hRad = CalculateKernelBilateralFilter(noisyImg.at<uchar>(neighbourX, neighbourY) - noisyImg.at<uchar>(i, j), sigmaRad);
						wtCombined = hSpat * hRad;
						imgFilt.second = wtCombined;
						imgFilt.first = noisyImg.at<uchar>(neighbourX, neighbourY);
						imgFiltVec.push_back(imgFilt);
					}
				}
			}
			/*sort the filtered vector*/
			sort(imgFiltVec.begin(), imgFiltVec.end(), greater<pair<uchar, float>>());
			/*calculate the weighted median*/
			for (auto& val : imgFiltVec) {
				accWt += val.second;
			}
			for (auto& val : imgFiltVec) {
				if (filtVal <= accWt / 2) {
					filtVal += val.second;
					imgVal = val.first;
				}
				else {
					break;
				}
			}
			filtImg.at<uchar>(i, j) = imgVal;
		}
	}
	return filtImg.clone();
}

void Bilateral::UpsamplingMenu() {
	int choice = 1;
	cout << "Upsampling Menu" << endl;
	cout << "1. Straighforward upsampling" << endl;
	cout << "2. Itrative upsampling" << endl;
	cin >> choice;
	switch (choice) {
	case 1:
		this->StraighforwardUpsampling();
		break;
	case 2:
		this->IterativeUpsampling();
		break;
	default:
		cout << "Enter valid choice" << endl;
		break;
	}
}

/* https://johanneskopf.de/publications/jbu/paper/FinalPaper_0185.pdf */
/*Joint Bilateral Median Filtering*/
Mat Bilateral::JointBilateralFiltering(Mat& hiRes, Mat& loRes, float sigmaSpat, float sigmaRad) {
	Mat filtImg = Mat::ones(loRes.size(), loRes.type());
	//float imgFilt = 0.;
	//float accWt = 0.;
	float hSpat = 1., hRad = 1.;
	float wtCombined = 1.;
	int neighbourX = 0, neighbourY = 0;
	int half = this->kSize / 2;
	for (int i = 0; i < loRes.rows; i++) {
		for (int j = 0; j < loRes.cols; j++) {
			//cout << i << " " << j << endl;
			vector<pair<uchar, float>> imgFiltVec;
			float accWt = 0., filtVal = 0.;
			uchar imgVal = 0;
			for (int row = 0; row < this->kSize; row++)
			{
				for (int col = 0; col < this->kSize; col++)
				{
					neighbourX = i + row;
					neighbourY = j + col;
					pair<uchar, float> imgFilt;
					if (not(neighbourX < 0 or neighbourX >= loRes.rows or neighbourY < 0 or neighbourY >= loRes.cols)) {
						//cout << neighbourX << " " << neighbourX << endl;
						/*Idea is to apply spatial filter to low resolution solution*/
						hSpat = CalculateKernelBilateralFilter(distance(i, j, neighbourX, neighbourY), sigmaSpat);
						/*Range filter is joitly applied to the full resolution (guidance) image*/
						hRad = CalculateKernelBilateralFilter(hiRes.at<uchar>(neighbourX, neighbourY) - hiRes.at<uchar>(i, j), sigmaRad);
						wtCombined = hSpat * hRad;
						imgFilt.second = wtCombined;
						imgFilt.first = loRes.at<uchar>(neighbourX, neighbourY);
						imgFiltVec.push_back(imgFilt);
					}
				}
			}
			/*sort the filtered vector*/
			sort(imgFiltVec.begin(), imgFiltVec.end(), greater<pair<uchar, float>>());
			/*calculate the weighted median*/
			for (auto& val : imgFiltVec) {
				accWt += val.second;
			}
			for (auto& val : imgFiltVec) {
				if (filtVal <= accWt / 2) {
					filtVal += val.second;
					imgVal = val.first;
				}
				else {
					break;
				}
			}
			filtImg.at<uchar>(i, j) = imgVal;
		}
	}
	return filtImg.clone();
}

/*Straighforward upsampling - create a low resolution image and use the high resolution image
for guided upsampling with joint bilateral median filtering*/

void Bilateral::StraighforwardUpsampling() {
	Mat resizeDownImg, upSampledImg;
	Utility utility;
	float sigmaSpat, sigmaRad;
	sigmaRad = 2 * this->kSize;
	sigmaSpat = this->kSize / 2;
	/*creating low resolution image from the high resolution disp ground truth*/
	resize(this->downImg, resizeDownImg, Size(), 0.25, 0.25, INTER_NEAREST);
	resize(resizeDownImg, resizeDownImg, Size(this->downImg.cols, this->downImg.rows), 0, 0);

	upSampledImg = this->JointBilateralFiltering(this->guidedImg, resizeDownImg, sigmaSpat, sigmaRad);

	imshow("ResizedDownsampled", resizeDownImg);
	imwrite("Straightforward_Downsampled.png", resizeDownImg);
	imshow("Straightforward_Upsampled", upSampledImg);
	imwrite("Straightforward_Upsampled.png", upSampledImg);
	waitKey(0);
}

/*Iterative upsampling*/
void Bilateral::IterativeUpsampling() {
	Mat resizeDownImg, upsampledImg, iterDepthImg, iterGuideImg;
	float sigmaSpat, sigmaRad;
	sigmaRad = 2 * this->kSize;
	sigmaSpat = this->kSize / 2;
	int upsampleFactor = 3;			//for an image downsized to 16 times the original size
	/*downsample the image*/
	float downsampleFactor = (1 / pow(2, upsampleFactor));
	float newSize = downsampleFactor;
	resize(this->downImg, resizeDownImg, Size(), downsampleFactor, downsampleFactor, INTER_LINEAR);
	iterDepthImg = resizeDownImg.clone();

	for (int i = 1; i <= (upsampleFactor - 1); i++) {
		newSize = pow(2, i);
		resize(iterDepthImg, iterDepthImg, Size(), 2, 2);
		resize(this->guidedImg, iterGuideImg, Size(iterDepthImg.cols, iterDepthImg.rows), 0, 0);
		iterDepthImg = this->JointBilateralFiltering(iterGuideImg, iterDepthImg, sigmaSpat, sigmaRad);
	}
	resize(iterDepthImg, iterDepthImg, Size(this->guidedImg.cols, this->guidedImg.rows), 0, 0);
	upsampledImg = this->JointBilateralFiltering(this->guidedImg, iterDepthImg, sigmaSpat, sigmaRad);

	imshow("IterDownsampled", iterDepthImg);
	imshow("Upsampled", upsampledImg);
	imwrite("Iterative_Downsampled.png", iterDepthImg);
	imwrite("Iterative_Upsampled.png", upsampledImg);
	waitKey(0);
}

