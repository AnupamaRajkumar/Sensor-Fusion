
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

void CloudRegistration::AddNoiseToData(allPtCloud& dataPCL)
{
	Mat noiseRotation = Mat::zeros(3, 3, CV_64F);
	Mat noiseTranslation = Mat::zeros(3, 1, CV_64F);

	/*add noisy rotation of 30 degs along z axis*/
	double angle = 45. * (M_PI / 180.);
	noiseRotation.at<double>(0, 0) = cos(angle);
	noiseRotation.at<double>(0, 1) = -sin(angle);
	noiseRotation.at<double>(0, 2) = 0.;
	noiseRotation.at<double>(1, 0) = sin(angle);
	noiseRotation.at<double>(1, 1) = cos(angle);
	noiseRotation.at<double>(1, 2) = 0.;
	noiseRotation.at<double>(2, 0) = 0.;
	noiseRotation.at<double>(2, 1) = 0.;
	noiseRotation.at<double>(2, 2) = 1.0;

	/*add random translation*/
	noiseTranslation.at<double>(0, 0) = 0.5;
	noiseTranslation.at<double>(0, 0) = 0.5;
	noiseTranslation.at<double>(0, 0) = 1.;
	/*apply randomtransformations to the datapoint*/
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		Mat dataPt = Mat::zeros(3, 1, CV_64F);
		dataPt.at<double>(0, 0) = this->dataPCL.pts[i].dataPt.x;
		dataPt.at<double>(1, 0) = this->dataPCL.pts[i].dataPt.y;
		dataPt.at<double>(2, 0) = this->dataPCL.pts[i].dataPt.z;
		dataPt = noiseRotation * dataPt;
		dataPt += noiseTranslation;
		this->dataPCL.pts[i].dataPt.x = dataPt.at<double>(0, 0);
		this->dataPCL.pts[i].dataPt.y = dataPt.at<double>(1, 0);
		this->dataPCL.pts[i].dataPt.z = dataPt.at<double>(2, 0);
	}
	string fileName = "NoisyDataSet.xyz";
	this->WriteDataPoints(dataPCL, fileName);
}

void CloudRegistration::IterativeClosestPoint() {
	/*start the timer*/
	auto start = std::chrono::high_resolution_clock::now();
	/*step 1 : load model and data point clouds*/
	cout << "Loading point clouds" << endl;
	/*model PCL*/
	this->LoadData(this->modelPCL, this->modelPCLFile);
	cout << "Model point cloud loaded, point cloud size:" << this->modelPCL.pts.size() << endl;
	/*data PCL*/
	this->LoadData(this->dataPCL, this->dataPCLFile);
	cout << "Data point cloud loaded, point cloud size:" << this->dataPCL.pts.size() << endl;
	//this->AddNoiseToData(this->dataPCL);
	//cout << "Random noise added to the data point cloud" << endl;
	int iterations = 1;
	double oldError = 0.0;
	while ((iterations <= this->maxIterations)) {			//  && (abs(oldError - this->error) > this->minThreshold)
		cout << "iteration number: " << iterations << endl;
		cout << "Difference in error: " << (oldError - this->error) << endl;
		oldError = this->error;
		/*step 2 : Data association - for each point in the data set, find the nearest neighbor*/
		this->FindNearestNeighbor();
		cout << "Found nearest points with count:" << this->nearestPts.size() << endl;
		/*sort the nearestPts wrt the first index in ascending order because parallel loop
		can shuffle the indices */
		sort(this->nearestPts.begin(), this->nearestPts.end());
		/*step 3 : Data transformation - from R and T matrix to tranform data set close to model set*/
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



/************************************************************
Trimmed iterative closest point (TrICP)
1. Find closest point in the model set such that  |M - S| = min using kd-tree
2. Trimmed squares : sort the distances, select Npo least values and calculate their sum
3. Convergence test : Repeat until any of the stopping conditions is satisfied
4. Motion calculation : Compute optimal motion (R, t) that minimises Sts
5. Data set motion
**************************************************************/


/*calculate the euclideam difference between the transformed matrix and the */
double CloudRegistration::CalculateDistanceError() {
	double error = 0;
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		Mat dataPt = Mat::zeros(3, 1, CV_64F);
		Mat modelPt = Mat::zeros(3, 1, CV_64F);
		dataPt.at<double>(0, 0) = dataPCL.pts[i].dataPt.x;
		dataPt.at<double>(1, 0) = dataPCL.pts[i].dataPt.y;
		dataPt.at<double>(2, 0) = dataPCL.pts[i].dataPt.z;
		modelPt.at<double>(0, 0) = modelPCL.pts[i].dataPt.x;
		modelPt.at<double>(1, 0) = modelPCL.pts[i].dataPt.y;
		modelPt.at<double>(2, 0) = modelPCL.pts[i].dataPt.z;
		error += norm((this->Rotation * dataPt + this->Translation) - modelPt);
	}
	error /= dataPCL.pts.size();
	cout << "error between transformed point and the closest point:" << error << endl;
	return error;
}

void CloudRegistration::CalculateTransformationMatrix() {
	/*step 1 : find center of mass of both the datasets*/
	Point3d pclCOM, mclCOM;
	Mat R, T;
	R = Mat::zeros(3, 3, CV_64F);
	T = Mat::zeros(3, 1, CV_64F);
	/*for data point cloud*/
	for (int i = 0; i < dataPCL.pts.size(); i++) {
		pclCOM += dataPCL.pts[i].dataPt;
	}
	pclCOM = pclCOM * (1.0 / dataPCL.pts.size());
	/*for nearest model point clouds*/
	for (int i = 0; i < modelPCL.pts.size(); i++) {
		mclCOM += modelPCL.pts[i].dataPt;
	}
	mclCOM = mclCOM * (1.0 / modelPCL.pts.size());
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
	for (int i = 0; i < modelPCL.pts.size(); i++) {
		Point3d pt;
		pt = modelPCL.pts[i].dataPt - mclCOM;				//nearestPts[i].second
		centerMCL.emplace_back(pt);
	}

	/*step 3 : calculate covariance matrix*/
	double vectorSize = nearestPts.size();
	Mat covariance = Mat::zeros(3, 3, CV_64F);
	double sumXX = 0., sumXY = 0., sumXZ = 0.;
	double sumYX = 0., sumYY = 0., sumYZ = 0.;
	double sumZX = 0., sumZY = 0., sumZZ = 0.;
	for (int i = 0; i < vectorSize; i++) {
		sumXX += centerPCL[i].x * centerMCL[nearestPts[i].second].x;
		sumXY += centerPCL[i].x * centerMCL[nearestPts[i].second].y;
		sumXZ += centerPCL[i].x * centerMCL[nearestPts[i].second].z;
		sumYX += centerPCL[i].y * centerMCL[nearestPts[i].second].x;
		sumYY += centerPCL[i].y * centerMCL[nearestPts[i].second].y;
		sumYZ += centerPCL[i].y * centerMCL[nearestPts[i].second].z;
		sumZX += centerPCL[i].z * centerMCL[nearestPts[i].second].x;
		sumZY += centerPCL[i].z * centerMCL[nearestPts[i].second].y;
		sumZZ += centerPCL[i].z * centerMCL[nearestPts[i].second].z;
	}
	covariance.at<double>(0, 0) = sumXX / vectorSize;
	covariance.at<double>(0, 1) = sumXY / vectorSize;
	covariance.at<double>(0, 2) = sumXZ / vectorSize;
	covariance.at<double>(1, 0) = sumYX / vectorSize;
	covariance.at<double>(1, 1) = sumYY / vectorSize;
	covariance.at<double>(1, 2) = sumYZ / vectorSize;
	covariance.at<double>(2, 0) = sumZX / vectorSize;
	covariance.at<double>(2, 1) = sumZY / vectorSize;
	covariance.at<double>(2, 2) = sumZZ / vectorSize;

	cout << "Covariance matrix" << endl;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			cout << covariance.at<double>(r, c) << " ";
		}
		cout << endl;
	}

#if SVD_REGISTRATION

	/*SVD registrayion method*/
	/*step 4: eigenvalues and eigenvectors from covariance matrix*/
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

	/*step 5: calculate rotation matrix*/
	R = vt.t() * u.t();
	cout << "determinant of R: " << determinant(R) << endl;
	if (determinant(R) < 0.) {
		cout << "Reflection detected..." << endl;
		vt.at<double>(2, 0) *= -1.;
		vt.at<double>(2, 1) *= -1.;
		vt.at<double>(2, 2) *= -1.;
		R = vt.t() * u.t();
	}

	cout << "Rotation matrix calculated" << endl;
	for (int r = 0; r < R.rows; r++) {
		for (int c = 0; c < R.cols; c++) {
			cout << R.at<double>(r, c) << " ";
		}
		cout << endl;
	}
#endif


#if QUAT_REGISTRATION

	/*Quaternion registration method - this method is used when SVD leads to reflections instead
	of rotation which could be due to outliers in the point cloud*/
	/*step 4a : Find cyclic components of anti-symmetric matrix AntiSymm*/
	Mat antiSymm = Mat::zeros(3, 3, CV_64F);
	antiSymm = covariance - covariance.t();

	/*step 4b : Calculate delta vector*/
	Mat delta = Mat::zeros(3, 1, CV_64F);
	delta.at<double>(0, 0) = antiSymm.at<double>(1, 2);
	delta.at<double>(1, 0) = antiSymm.at<double>(2, 0);
	delta.at<double>(2, 0) = antiSymm.at<double>(0, 1);

	/*step 5 : Form 4x4 symmetric matrix Q*/
	Mat Q = Mat::zeros(4, 4, CV_64F);
	Mat temp = Mat::zeros(3, 3, CV_64F);
	Mat traceEye = temp.clone();
	double covTrace = covariance.at<double>(0, 0) + covariance.at<double>(1, 1) + covariance.at<double>(2, 2);
	for (int r = 0; r < traceEye.rows; r++) {
		for (int c = 0; c < traceEye.cols; c++) {
			if (r == c) {
				traceEye.at<double>(r, c) = covTrace;
			}
		}
	}
	for (int r = 0; r < traceEye.rows; r++) {
		for (int c = 0; c < traceEye.cols; c++) {
			cout << traceEye.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	temp = covariance + covariance.t() - traceEye;
	Q.at<double>(0, 0) = covTrace;
	Q.at<double>(0, 1) = delta.at<double>(0, 0);
	Q.at<double>(0, 2) = delta.at<double>(1, 0);
	Q.at<double>(0, 3) = delta.at<double>(2, 0);

	Q.at<double>(1, 0) = delta.at<double>(0, 0);
	Q.at<double>(1, 1) = temp.at<double>(0, 0);
	Q.at<double>(1, 2) = temp.at<double>(0, 1);
	Q.at<double>(1, 3) = temp.at<double>(0, 2);

	Q.at<double>(2, 0) = delta.at<double>(1, 0);
	Q.at<double>(2, 1) = temp.at<double>(1, 0);
	Q.at<double>(2, 2) = temp.at<double>(1, 1);
	Q.at<double>(2, 3) = temp.at<double>(1, 2);

	Q.at<double>(3, 0) = delta.at<double>(2, 0);
	Q.at<double>(3, 1) = temp.at<double>(2, 0);
	Q.at<double>(3, 2) = temp.at<double>(2, 1);
	Q.at<double>(3, 3) = temp.at<double>(2, 2);

	/*step 6 : Find eigenvector for max eigenvalue in Q*/
	Mat eVals, eVecs;
	eigen(Q, eVals, eVecs);
	cout << "Eigen value" << endl;
	for (int r = 0; r < eVals.rows; r++) {
		for (int c = 0; c < eVals.cols; c++) {
			cout << eVals.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	cout << "Eigen vector" << endl;
	for (int r = 0; r < eVecs.rows; r++) {
		for (int c = 0; c < eVecs.cols; c++) {
			cout << eVecs.at<double>(r, c) << " ";
		}
		cout << endl;
	}
	/*step 7 : Compute rotation matrix*/
	/*since eigenvalues are sorted in descending order, the first column on eVecs corresponds to max eVal*/
	/*qr = [eVecs[0][0], eVecs[1][0], eVecs[2][0], eVecs[3][0]]*/
	double q0 = eVecs.at<double>(0, 0);
	double q1 = eVecs.at<double>(1, 0);
	double q2 = eVecs.at<double>(2, 0);
	double q3 = eVecs.at<double>(3, 0);
	R.at<double>(0, 0) = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	R.at<double>(0, 1) = 2 * (q1 * q2 - q0 * q3);
	R.at<double>(0, 2) = 2 * (q1 * q3 + q0 * q2);

	R.at<double>(1, 0) = 2 * (q1 * q2 + q0 * q3);
	R.at<double>(1, 1) = q0 * q0 + q2 * q2 - q1 * q1 - q3 * q3;
	R.at<double>(1, 2) = 2 * (q2 * q3 - q0 * q1);

	R.at<double>(2, 0) = 2 * (q1 * q3 - q0 * q2);
	R.at<double>(2, 1) = 2 * (q2 * q3 + q0 * q1);
	R.at<double>(2, 2) = q0 * q0 + q3 * q3 - q1 * q1 - q2 * q2;

	cout << "R matrix:" << endl;
	for (int r = 0; r < R.rows; r++) {
		for (int c = 0; c < R.cols; c++) {
			cout << R.at<double>(r, c) << " ";
		}
		cout << endl;
	}

#endif // QUAT_REGISTRATION

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
void CloudRegistration::FindNearestNeighbor() {	

#pragma omp parallel for
	for (int i = 0; i < dataPCL.pts.size(); i++) {
			//cout << i << endl;
			if ((this->nearestPts.size() > 0) && (this->nearestPts.size() % 10000) == 0)
				cout << this->nearestPts.size() << " neighbors found" << endl;
			//Find minimum distance from points in the model set
			double  query_pt[3] = { dataPCL.pts[i].dataPt.x, dataPCL.pts[i].dataPt.y, dataPCL.pts[i].dataPt.z };
			myKDTree index(3, modelPCL, KDTreeSingleIndexAdaptorParams(10));
			index.buildIndex();
			size_t ret_index;
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
			pair<int, size_t> nearComb;
			nearComb.first = i;
			nearComb.second = ret_index;
			this->nearestPts.push_back(nearComb);
	}
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
