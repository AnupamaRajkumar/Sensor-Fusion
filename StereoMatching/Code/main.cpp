/************************************************************************************************
A. Stereo matching techniques:
1. Naive stereo matching
2. Dynamic programming approach - make parameters tunable
B. Display disparity images
C. Save disparity images to 3D point cloud
Extra points : parallelising/fast implementation, compare solution to existing implementation
**************************************************************************************************/


#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> 



#include "DynamicApproach.h"
#include "NaiveApproach.h"
#include "Utility.h"

using namespace std;
using namespace cv;


int main(int argc, char** argv) {

  ////////////////
  // Parameters //
  ////////////////

  // camera setup parameters
  const double focal_length = 1247;
  const double baseline = 213;

  // stereo estimation parameters
  const int dmin = 67;
  int window_size = 7;
  double weight = 500;
  const double scale = 3;

  ///////////////////////////
  // Commandline arguments //
  ///////////////////////////

  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " IMAGE1 IMAGE2 " << std::endl;
    return 1;
  }

  Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

  if (!image1.data) {
    cerr << "No image1 data" << std::endl;
    return EXIT_FAILURE;
  }

  if (!image2.data) {
    cerr << "No image2 data" << std::endl;
    return EXIT_FAILURE;
  }

  cout << "Enter the window size:" << endl;
  cin >> window_size;
  cout << "Enter the occlusion weights:" << endl;
  cin >> weight;

  cout << "------------------ Parameters -------------------" << std::endl;
  cout << "focal_length = " << focal_length << std::endl;
  cout << "baseline = " << baseline << std::endl;
  cout << "window_size = " << window_size << std::endl;
  cout << "occlusion weights = " << weight << std::endl;
  cout << "disparity added due to image cropping = " << dmin << std::endl;
  cout << "scaling of disparity images to show = " << scale << std::endl;
  cout << "-------------------------------------------------" << std::endl;


  /*class instantiation*/
  Utility utility;
  Naive naive(window_size, image1, image2, dmin, focal_length, baseline);

  int choice = 1;
  ////////////////////
  // Reconstruction menu //
  ////////////////////
  while (choice != 5) {

	  cout << "Reconstruction Menu:" << endl;
	  cout << "1. Naive Approach" << endl;
	  cout << "2. Dynamic Programming Approach" << endl;
	  cout << "3. OpenCV stereo matching methods" << endl;
	  cout << "4. Generation of point clouds" << endl;
	  cout << "5. Exit" << endl;
	  cout << "Enter your choice:" << endl;
	  cin >> choice;
	  cout << "----------------------------------------------------" << endl;
	
	  stringstream out1;
	  Mat naive_disparities = cv::Mat::zeros(image1.size().height, image1.size().width, CV_8UC1);
	  switch (choice) {
		case 1:
			// Naive disparity image		
			//utility.StereoEstimation_Naive(window_size, dmin, image1, image2, naive_disparities, scale);
			naive.NaiveMatchingCalculation();
			// save / display images
			/*out1 << output_file << "_naive.png";
			imwrite(out1.str(), naive_disparities);
			namedWindow("Naive", cv::WINDOW_AUTOSIZE);
			imshow("Naive", naive_disparities);*/
			break;
		case 2:
			/*openCV stereo matching methods*/
		case 3:
			/*Dynamic programming approach*/
			break;
		case 4:
			// reconstruction
			/*utility.Disparity2PointCloud(output_file, image1.size().height, image1.size().width, naive_disparities,
				window_size, dmin, baseline, focal_length);*/
			break;
		case 5:
			exit(0);
			break;
		default:
			cout << "Enter correct choice" << endl;
			break;
	  }
  }

  waitKey(0);

  return 0;
}
