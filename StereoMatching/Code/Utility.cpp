#include "Utility.h"


void Utility::Disparity2PointCloud(std::string& output_file, cv::Mat& disparities, int& window_size, int& dmin, 
									double& baseline, double& focal_length, double& cx_d, double& cy_d, double doffs)
{
	std::stringstream out3d;
	out3d << output_file << ".xyz";
	std::ofstream outfile(out3d.str());
	cv::Mat pxToMetric = cv::Mat::zeros(disparities.size(), CV_32F);
	ConvertPixelToMetric(disparities, pxToMetric, dmin, baseline, focal_length, doffs);
	for (int x = 0; x < pxToMetric.rows; ++x) {
		std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((x) / static_cast<double>(disparities.rows - window_size + 1)) * 100) << "%\r" << std::flush;
		for (int y = 0; y < pxToMetric.cols; ++y) {
			if (pxToMetric.at<float>(x, y) == 0) 
				continue;
			double Z = pxToMetric.at<float>(x, y);
			double X = (x - cx_d) * Z / focal_length;
			double Y = (y - cy_d) * Z / focal_length;
			outfile << X << " " << Y << " " << Z << std::endl;
		}
	}
	std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
	std::cout << std::endl;
}

void Utility::saveDisparityImage(std::string& fileName, cv::Mat& disparity) {
	cv::imwrite(fileName, disparity);
}

void Utility::ConvertPixelToMetric(cv::Mat& disparities, cv::Mat& pxToMetric, int dmin, 
								   double baseline, double focal_length, double doffs) {
	for (int r = 0; r < disparities.rows; r++) {
		for (int c = 0; c < disparities.cols; c++) {
			int disparity = int(disparities.at<uchar>(r, c));
			if (disparity != 255) {
				int disp = disparity * (dmin / 255);
				double pxDist = (baseline * focal_length) / (disp + doffs);
				pxToMetric.at<float>(r, c) = pxDist;
			}
			else {
				pxToMetric.at<float>(r, c) = 0.;
			}
		}
	}
}