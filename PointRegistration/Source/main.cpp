/****************************************************************************
1. Choose appropriate mesh clouds - cloud1.xyz and cloud2.xyz in our case
	- If input points are well aligned, rotate and displace one of them, randomly
2. Perform nearest neighbor search using kd-trees (nanoflann)
3. Implement ICP
4. Implement Tr-ICP
Evaluation:
1. Test various noise levels (add gaussian noise to 3D point locations, synthetically)
2. Measure alignment precision (angular- rotation error, euclidean - translation error)
3. Measure runtime
4. Plot results (diagrams)
5. Compare ICP to Tr-ICP and also PCL
*****************************************************************************/

#include "IterativeClosestPoint.h"

int main(int argc, char** argv)
{
	if (argc < 3) {
		cerr << "Input format : Model point cloud, data point cloud" << endl;
		return EXIT_FAILURE;
	}

	CloudRegistration cloudRegister(argv[1], argv[2]);

	waitKey(0);
	return 0;
}



