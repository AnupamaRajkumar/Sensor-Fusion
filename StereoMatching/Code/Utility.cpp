#include "Utility.h"

using namespace std;
using namespace cv;

void Utility::StereoEstimation_Naive(const int& window_size, const int& dmin,
	Mat& image1, Mat& image2, cv::Mat& naive_disparities, const double& scale)
{
	int half_window_size = window_size / 2;
	int height = image1.size().height;
	int width = image1.size().width;
	/*vertical direction*/
	for (int i = half_window_size; i < height - half_window_size; ++i) {

		std::cout
			<< "Calculating disparities for the naive approach... "
			<< std::ceil(((i - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
			<< std::flush;
		/*horizontal direction*/
		for (int j = half_window_size; j < width - half_window_size; ++j) {
			int min_ssd = INT_MAX;
			int disparity = 0;
			/*disparity in horizontal direction*/
			for (int d = -j + half_window_size; d < width - j - half_window_size; ++d) {
				int ssd = 0;

				// TODO: sum up matching cost (ssd) in a window
					/*window around each pixel*/
				for (int u = -half_window_size; u <= half_window_size; u++) {
					for (int v = -half_window_size; v <= half_window_size; v++) {
						int u1 = i + u;
						int v1 = j + v;

						int u2 = i + u;
						int v2 = j + v + d;

						ssd += pow((image1.at<uchar>(u1, v1) - image2.at<uchar>(u2, v2)), 2);
					}
				}

				if (ssd < min_ssd) {
					min_ssd = ssd;
					disparity = d;
				}
			}

			naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity) * scale;
		}
	}

	std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
	std::cout << std::endl;
}


/*******************************************************************
Apply parallelism - parallel for, SIMD, Cuda
********************************************************************/

void Utility::StereoEstimation_NaiveFast(const int& window_size, const int& dmin,
	Mat& image1, Mat& image2, cv::Mat& naive_disparities, const double& scale)
{
	int half_window_size = window_size / 2;
	int height = image1.size().height;
	int width = image1.size().width;
	/*vertical direction*/
	for (int i = half_window_size; i < height - half_window_size; ++i) {

		std::cout
			<< "Calculating disparities for the naive approach... "
			<< std::ceil(((i - half_window_size + 1) / static_cast<double>(height - window_size + 1)) * 100) << "%\r"
			<< std::flush;
		/*horizontal direction*/
		for (int j = half_window_size; j < width - half_window_size; ++j) {
			int min_ssd = INT_MAX;
			int disparity = 0;
			/*disparity in horizontal direction*/
			for (int d = -j + half_window_size; d < width - j - half_window_size; ++d) {
				int ssd = 0;

				// TODO: sum up matching cost (ssd) in a window
				/*window around each pixel*/
//#pragma omp parallel for
				for (int u = -half_window_size; u <= half_window_size; u++) {
					for (int v = -half_window_size; v <= half_window_size; v++) {
						int u1 = i + u;
						int v1 = j + v;

						int u2 = i + u;
						int v2 = j + v + d;

						ssd += pow((image1.at<uchar>(u1, v1) - image2.at<uchar>(u2, v2)), 2);
					}
				}

				/*parallel_for(size_t(0), half_window_size, [&](size_t i)
				{
					for (size_t u = -half_window_size; u <= half_window_size; u++)
					{
						for (int v = -half_window_size; v <= half_window_size; v++)
						{
							int u1 = i + u;
							int v1 = j + v;

							int u2 = i + u;
							int v2 = j + v + d;

							ssd += pow((image1.at<uchar>(u1, v1) - image2.at<uchar>(u2, v2)), 2);
						}
					}
				});*/

				if (ssd < min_ssd) {
					min_ssd = ssd;
					disparity = d;
				}
			}
			naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity) * scale;
		}
	}

	std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
	std::cout << std::endl;
}


void Utility::Disparity2PointCloud(const std::string& output_file, int height, int width, cv::Mat& disparities, const int& window_size,
	const int& dmin, const double& baseline, const double& focal_length)
{
	std::stringstream out3d;
	out3d << output_file << ".xyz";
	std::ofstream outfile(out3d.str());
	for (int i = 0; i < height - window_size; ++i) {
		std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
		for (int j = 0; j < width - window_size; ++j) {
			if (disparities.at<uchar>(i, j) == 0) continue;

			// TODO
			//const double Z = ...
			//const double X = ...
			//const double Y = ...
			//
			//outfile << X << " " << Y << " " << Z << std::endl;
		}
	}

	std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
	std::cout << std::endl;
}