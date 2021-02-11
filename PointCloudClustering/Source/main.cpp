#include "PointCloudClustering.h"

int main(int argc, char** argv)
{
	if (argc < 2) {
		std::cerr << "3D point cloud should be passed as an input" << std::endl;
		exit(-1);
	}
	char* fileName = argv[1];
	CloudClustering cluster(fileName);


	return 0;
}


