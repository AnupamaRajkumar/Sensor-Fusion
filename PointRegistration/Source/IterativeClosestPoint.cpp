
/*Iterative closest point:
1. Find closest point in the model set such that |M - S| = min using kd-tree
2. Find R and T at this point using SVD such that |R*S + T - M| is minimised
3. Rotate and translate the dataset point cloud
4. Calculate the error ie the distance between the transformed points with the model set
- if the error is less than a threshold, exit
- else continue the above steps until the error is less than the threshold
*/

#include "IterativeClosestPoint.h"

CloudRegistration::CloudRegistration(char* modelPCLFile, char* dataPCLFile) {
	this->modelPCLFile = modelPCLFile;
	this->dataPCLFile = dataPCLFile;
	/*Perform point cloud registration*/
	this->IterativeClosestPoint();
}

void CloudRegistration::IterativeClosestPoint() {
	int maxIterations = 100;
	double minThreshold = 0.1;
	/*step 1 : load model and data point clouds*/
	allPtCloud modelPCL, dataPCL;
	cout << "Loading point clouds" << endl;
	/*model PCL*/
	this->LoadData(modelPCL, this->modelPCLFile);
	cout << "Model point cloud loaded, point cloud size:" << modelPCL.pts.size() << endl;
	/*data PCL*/
	this->LoadData(dataPCL, this->dataPCLFile);
	cout << "Data point cloud loaded, point cloud size:" << dataPCL.pts.size() << endl;
	/*step 2 : Data association - for each point in the data set, find the nearest neighbor*/
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		cout << "data point ct : " << i << endl;
		size_t nearest = FindNearestNeighbor(dataPCL.pts[i], modelPCL);
	}
	cout << "Found nearest points" << endl;
}

/*Use nanoflann to find nearest neighbor*/
/* https://github.com/jlblancoc/nanoflann/blob/master/examples/pointcloud_example.cpp */
size_t CloudRegistration::FindNearestNeighbor(Point& queryPt, allPtCloud& modelPCL) {
	/*Find minimum distance from points in the model set*/
	double  query_pt[3] = { queryPt.x, queryPt.y, queryPt.z };
	size_t ret_index;
	myKDTree index(3 /*dim*/, modelPCL, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
	index.buildIndex();
	{
		// do a knn search
		const size_t num_results = 1;	
		double out_dist_sqr;
		nanoflann::KNNResultSet<double> resultSet(num_results);
		resultSet.init(&ret_index, &out_dist_sqr);
		index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

		//std::cout << "knnSearch(nn=" << num_results << "): \n";
		//std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;
	}
	return ret_index;
}

void CloudRegistration::LoadData(allPtCloud& points, char* fileName) {
	ifstream datafile(fileName);
	string line;
	int lineCounter = 0;
	if (datafile.is_open()) {
		while (!datafile.eof()) {
			vector<string> coords;
			string word = " ";
			getline(datafile, line);
			for (char l : line) {
				if (l == ' ') {
					//cout << word << endl;
					coords.push_back(word);
					word = "";
				}
				else {
					word = word + l;
				}
			}
			Point p;
			//cout << word << endl;
			coords.push_back(word);
			size_t sz;
			/*Point coordinates - x, y, z*/
			p.x = stod(coords[0], &sz);
			p.y = stod(coords[1], &sz);
			p.z = stod(coords[2], &sz);
			/*Point normals - nx, ny, nz*/
			p.nx = stod(coords[3], &sz);
			p.ny = stod(coords[4], &sz);
			p.nz = stod(coords[5], &sz);
			
			points.pts.emplace_back(p);
		}
	}
	else {
		cerr << "Cannot open file" << endl;
		exit(-1);
	}
}