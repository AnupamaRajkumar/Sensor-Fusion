#include "Utility.h"


void Utility::Disparity2PointCloud(std::string& output_file, int height, int width, cv::Mat& disparities, int& window_size,
									int& dmin, double& baseline, double& focal_length)
{
	std::stringstream out3d;
	out3d << output_file << ".xyz";
	std::ofstream outfile(out3d.str());
	for (int i = 0; i < height - window_size; ++i) {
		std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
		for (int j = 0; j < width - window_size; ++j) {
			if (disparities.at<uchar>(i, j) == 0) continue;

			//double Z = (baseline * focal_length) / disparities.at<uchar>(i, j);
			//double X = -baseline * () / 2 * disparities.at<uchar>(i, j);
			//double Y = baseline * / disparities.at<uchar>(i, j);
			//outfile << X << " " << Y << " " << Z << std::endl;
		}
	}
	std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
	std::cout << std::endl;
}

void Utility::saveDisparityImage(std::string& fileName, cv::Mat& disparity) {
	cv::imwrite(fileName, disparity);
}