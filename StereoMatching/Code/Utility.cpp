#include "Utility.h"


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

void Utility::saveDisparityImage(std::string& fileName, cv::Mat& disparity) {
	cv::imwrite(fileName, disparity);
}