
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
	/*start the timer*/
	auto start = std::chrono::high_resolution_clock::now();
	/*step 1 : load model and data point clouds*/
	cout << "Loading point clouds" << endl;
	/*model PCL*/
	this->LoadData(modelPCL, this->modelPCLFile);
	cout << "Model point cloud loaded, point cloud size:" << modelPCL.pts.size() << endl;
	/*data PCL*/
	this->LoadData(dataPCL, this->dataPCLFile);
	cout << "Data point cloud loaded, point cloud size:" << dataPCL.pts.size() << endl;
	int iterations = 1;
	double oldError = 0.0;
	while (iterations <= this->maxIterations) {
		cout << "iteration number: " << iterations << endl;
		cout << "Difference in error: " << abs(oldError - this->error) << endl;
		oldError = this->error;
		/*step 2 : Data association - for each point in the data set, find the nearest neighbor*/
#pragma omp parallel for
		for (int i = 0; i < dataPCL.pts.size(); i++) {
			if ((nearestPts.size() > 0) && (nearestPts.size() % 10000) == 0)
				cout << nearestPts.size() << " neighbors found" << endl;

			size_t nearest = this->FindNearestNeighbor(dataPCL.pts[i], modelPCL);
			pair<int, size_t> nearComb;
			nearComb.first = i;
			nearComb.second = nearest;
			nearestPts.push_back(nearComb);
		}
		cout << "Found nearest points with count:" << nearestPts.size() << endl;
		/*sort the nearestPts wrt the first index in ascending order because parallel loop
		can shuffle the indices */
		sort(nearestPts.begin(), nearestPts.end());
		/*step 3 : Data transformation - from R and T matrix to tranform data set close to model set*/
		Rotation = Mat::zeros(3, 3, CV_64F);
		Translation = Mat::zeros(3, 1, CV_64F);
		this->CalculateTransformationMatrix();

		/*apply transformations to the datapoint*/
		for (int i = 0; i < dataPCL.pts.size(); i++) {
			Mat dataPt = Mat::zeros(3, 1, CV_64F);
			dataPt.at<double>(0, 0) = dataPCL.pts[i].dataPt.x;
			dataPt.at<double>(1, 0) = dataPCL.pts[i].dataPt.y;
			dataPt.at<double>(2, 0) = dataPCL.pts[i].dataPt.z;
			dataPt = this->Rotation * dataPt;
			dataPt += this->Translation;
			dataPCL.pts[i].dataPt.x = dataPt.at<double>(0, 0);
			dataPCL.pts[i].dataPt.y = dataPt.at<double>(1, 0);
			dataPCL.pts[i].dataPt.z = dataPt.at<double>(2, 0);
		}

		/*calculate the error*/
		this->error = this->CalculateDistanceError();
		nearestPts.clear();
		iterations++;
	}
	/*end the timer*/
	auto finish = std::chrono::high_resolution_clock::now();
	/*calculate the time needed for the whole process*/
	std::chrono::duration<double> elapsed = (finish - start);
	cout << "Time taken = " << elapsed.count()/60. << " minutes" << endl;
	/*save the point cloud*/
	cout << "Saving the registered point cloud....." << endl;
	string fileName = "RegisteredData.xyz";
	this->WriteDataPoints(dataPCL, fileName);
}

/*calculate the euclideam difference between the transformed matrix and the */
double CloudRegistration::CalculateDistanceError() {
	double error = 0;
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		Mat dataPt = Mat::zeros(3, 1, CV_64F);
		Mat modelPt = Mat::zeros(3, 1, CV_64F);
		dataPt.at<double>(0, 0) = dataPCL.pts[i].dataPt.x;
		dataPt.at<double>(1, 0) = dataPCL.pts[i].dataPt.y;
		dataPt.at<double>(2, 0) = dataPCL.pts[i].dataPt.z;
		modelPt.at<double>(0, 0) = modelPCL.pts[nearestPts[i].second].dataPt.x;
		modelPt.at<double>(1, 0) = modelPCL.pts[nearestPts[i].second].dataPt.y;
		modelPt.at<double>(2, 0) = modelPCL.pts[nearestPts[i].second].dataPt.z;
		error += norm((this->Rotation * dataPt + this->Translation) - modelPt);
	}
	error /= dataPCL.pts.size();
	cout << "error between transformed point and the closest point:" << error << endl;
	return error;
}

void CloudRegistration::CalculateTransformationMatrix() {
	/*step 1 : find center of mass of both the datasets*/
	Point3d pclCOM, mclCOM;
	/*for data point cloud*/
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		pclCOM += dataPCL.pts[i].dataPt;
	}
	pclCOM = pclCOM * (1.0 / dataPCL.pts.size());
	/*for nearest model point clouds*/
	for (int i = 0; i < nearestPts.size(); i++) {
		mclCOM += modelPCL.pts[nearestPts[i].second].dataPt;
	}
	mclCOM = mclCOM * (1.0 / nearestPts.size());
	cout << "pclCOM: " << pclCOM << " mclCOM: " << mclCOM << endl;
	/*step 2 : center the point cloud as per the center of mass calculated*/
	vector<Point3d> centerPCL, centerMCL;
	/*for data point cloud*/
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		Point3d pt;
		pt = dataPCL.pts[i].dataPt - pclCOM;
		centerPCL.emplace_back(pt);
	}
	/*for nearest model point cloud*/
	for (int i = 0; i < nearestPts.size(); i++) {
		Point3d pt;
		pt = modelPCL.pts[nearestPts[i].second].dataPt - mclCOM;
		centerMCL.emplace_back(pt);
	}

	/*step 3 : calculate covariance matrix*/
	double vectorSize = nearestPts.size();
	Mat covariance = Mat::zeros(3, 3, CV_64F);
	double sumXX = 0., sumXY = 0., sumXZ = 0.;
	double sumYX = 0., sumYY = 0., sumYZ = 0.;
	double sumZX = 0., sumZY = 0., sumZZ = 0.;
	for (int i = 0; i < vectorSize; i++) {
		sumXX += centerPCL[i].x * centerMCL[i].x;
		sumXY += centerPCL[i].x * centerMCL[i].y;
		sumXZ += centerPCL[i].x * centerMCL[i].z;
		sumYX += centerPCL[i].y * centerMCL[i].x;
		sumYY += centerPCL[i].y * centerMCL[i].y;
		sumYZ += centerPCL[i].y * centerMCL[i].z;
		sumZZ += centerPCL[i].z * centerMCL[i].x;
		sumXX += centerPCL[i].z * centerMCL[i].y;
		sumXX += centerPCL[i].z * centerMCL[i].z;
	}
	covariance.at<double>(0, 0) = sumXX;
	covariance.at<double>(0, 1) = sumXY;
	covariance.at<double>(0, 2) = sumXZ;
	covariance.at<double>(1, 0) = sumYZ;
	covariance.at<double>(1, 1) = sumYY;
	covariance.at<double>(1, 2) = sumYZ;
	covariance.at<double>(2, 0) = sumZX;
	covariance.at<double>(2, 1) = sumZY;
	covariance.at<double>(2, 2) = sumZZ;

	cout << "Covariance matrix" << endl;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			cout << covariance.at<double>(r, c) << " ";
		}
		cout << endl;
	}

	/*eigenvalues and eigenvectors from covariance matrix*/
	Mat w, u, vt;
	//SVDecomp(covariance, w, u, vt, 0);
	SVD::compute(covariance, w, u, vt, 0);
	cout << "OpenCV Eigen vector U" << endl;
	for (int r = 0; r < u.rows; r++) {
		for (int c = 0; c < u.cols; c++) {
			cout << u.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	cout << "OpenCV Eigen vector V'" << endl;
	for (int r = 0; r < vt.rows; r++) {
		for (int c = 0; c < vt.cols; c++) {
			cout << vt.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	cout << "OpenCV Eigen vector W" << endl;
	for (int r = 0; r < w.rows; r++) {
		for (int c = 0; c < w.cols; c++) {
			cout << w.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	cout << "determinant of U :" << determinant(u) << endl;
	cout << "determinant of V' :" << determinant(vt) << endl;
	Mat R, T;
	Mat v, ut;
	R = Mat::zeros(3, 3, CV_64F);
	T = Mat::zeros(3, 1, CV_64F);
	/*transpose U and V'*/
	transpose(u, ut);
	transpose(vt, v);
	/*calculate rotation matrix*/
	R = v * ut;
	cout << "determinant of R: " << determinant(R) << endl;
	if (determinant(R) < 0.) {
		cout << "Reflection detected..." << endl;
		vt.at<double>(2, 0) *= -1.;
		vt.at<double>(2, 1) *= -1.;
		vt.at<double>(2, 2) *= -1.;
		transpose(vt, v);
		R = v * ut;
	}

	cout << "Rotation matrix calculated" << endl;
	for (int r = 0; r < R.rows; r++) {
		for (int c = 0; c < R.cols; c++) {
			cout << R.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	/*calculate translation matrix*/
	Mat matPclCOM, matMclCOM;
	matPclCOM = matMclCOM = T.clone();
	matPclCOM.at<double>(0, 0) = pclCOM.x;
	matPclCOM.at<double>(1, 0) = pclCOM.y;
	matPclCOM.at<double>(2, 0) = pclCOM.z;

	matMclCOM.at<double>(0, 0) = mclCOM.x;
	matMclCOM.at<double>(1, 0) = mclCOM.y;
	matMclCOM.at<double>(2, 0) = mclCOM.z;

	T = matMclCOM - (R * matPclCOM);
	cout << "Translation matrix is:" << endl;
	for (int r = 0; r < T.rows; r++) {
		for (int c = 0; c < T.cols; c++) {
			cout << T.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	
	this->Rotation = R.clone();
	this->Translation = T.clone();
}

/*Use nanoflann to find nearest neighbor*/
/* https://github.com/jlblancoc/nanoflann/blob/master/examples/pointcloud_example.cpp */
size_t CloudRegistration::FindNearestNeighbor(Point& queryPt, allPtCloud& modelPCL) {
	/*Find minimum distance from points in the model set*/
	double  query_pt[3] = { queryPt.dataPt.x, queryPt.dataPt.y, queryPt.dataPt.z };
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
			Point3d dP, nP;
			//cout << word << endl;
			coords.push_back(word);
			size_t sz;
			/*Point coordinates - x, y, z*/
			dP.x = stod(coords[0], &sz);
			dP.y = stod(coords[1], &sz);
			dP.z = stod(coords[2], &sz);
			/*Point normals - nx, ny, nz*/
			nP.x = stod(coords[3], &sz);
			nP.y = stod(coords[4], &sz);
			nP.z = stod(coords[5], &sz);
			p = { dP, nP };
			points.pts.emplace_back(p);
		}
	}
	else {
		cerr << "Cannot open file" << endl;
		exit(-1);
	}
}

void CloudRegistration::WriteDataPoints(allPtCloud& points, string fileName) {
	ofstream dataFile;
	dataFile.open(fileName);
	cout << "Writing into xyz file" << endl;
	for (int i = 0; i < modelPCL.pts.size(); i++) {
		dataFile << modelPCL.pts[i].dataPt.x << " " << modelPCL.pts[i].dataPt.y << " " << modelPCL.pts[i].dataPt.z << " " <<
			modelPCL.pts[i].normalPt.x << " " << modelPCL.pts[i].normalPt.y << " " << modelPCL.pts[i].normalPt.z << " " << endl;
	}
	for (int i = 0; i < points.pts.size(); i++) {
		dataFile << points.pts[i].dataPt.x << " " << points.pts[i].dataPt.y << " " << points.pts[i].dataPt.z << " " <<
			points.pts[i].normalPt.x << " " << points.pts[i].normalPt.y << " " << points.pts[i].normalPt.z << " " << endl;
	}
	dataFile.close();
}