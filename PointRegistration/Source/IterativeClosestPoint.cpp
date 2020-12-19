
#include "IterativeClosestPoint.h"


CloudRegistration::CloudRegistration(char* modelPCLFile, char* dataPCLFile) {
	this->modelPCLFile = modelPCLFile;
	this->dataPCLFile = dataPCLFile;
	/*Perform initial steps, like data loading etc*/
	this->PreProcessingSteps();
	/*Perform point cloud registration*/
	this->PointRegistrationMenu();
}

CloudRegistration::~CloudRegistration() {
	this->nearestPts.clear();
	this->squareDist.clear();
	this->modelPCL.pts.clear();
	this->dataPCL.pts.clear();
	this->Rotation.release();
	this->Translation.release();
}

void CloudRegistration::PointRegistrationMenu() {
	int choice = 1;
	cout << "Point Registration Menu" << endl;
	cout << "1. Iterarative Closest Point" << endl;
	cout << "2. Trimmed Iterative Closest Point" << endl;
	cout << "Enter choice (1/2)" << endl;
	cin >> choice;
	switch (choice)
	{
	case 1:
		this->IterativeClosestPoint();
		break;
	case 2:
		this->TrimmedICP();
		break;
	default:
		break;
	}
}

#if ADD_NOISE
void CloudRegistration::AddNoiseToData(allPtCloud& dataPCL)
{
	Mat noiseRotation = Mat::zeros(3, 3, CV_64F);
	Mat noiseTranslation = Mat::zeros(3, 1, CV_64F);

	/*add noisy rotation of some degs along z axis*/
	double angle = 30. * (M_PI / 180.);
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
	noiseTranslation.at<double>(0, 0) = 0.;
	noiseTranslation.at<double>(0, 0) = 0.;
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

#endif

void CloudRegistration::PreProcessingSteps() {
	/*load model and data point clouds*/
	cout << "Loading point clouds" << endl;
	/*model PCL*/
	this->LoadData(this->modelPCL, this->modelPCLFile);
	cout << "Model point cloud loaded, point cloud size:" << this->modelPCL.pts.size() << endl;
	/*data PCL*/
	this->LoadData(this->dataPCL, this->dataPCLFile);
	cout << "Data point cloud loaded, point cloud size:" << this->dataPCL.pts.size() << endl;
#if ADD_NOISE
	/*Add noise to the point cloud by adding some random rotaton and translation*/
	cout << "Random noise added to the data point cloud" << endl;
	this->AddNoiseToData(this->dataPCL);
#endif
}

/*Iterative closest point:
1. Find closest point in the model set such that |M - S| = min using kd-tree
2. Find R and T at this point using SVD such that |R*S + T - M| is minimised
3. Rotate and translate the dataset point cloud
4. Calculate the error ie the distance between the transformed points with the model set
- if the error is less than a threshold, exit
- else continue the above steps until the error is less than the threshold
*/
void CloudRegistration::IterativeClosestPoint() {
	/*start the timer*/
	auto start = std::chrono::high_resolution_clock::now();
	int iterations = 1;
	double oldError = 0.0;
	bool step = true;
	while (step) {
		if ((iterations <= this->maxIterations) && !(abs(oldError - this->error) < this->minThreshold)) {
			cout << "iteration number: " << iterations << endl;
			/*step 2 : Data association - for each point in the data set, find the nearest neighbor*/
			this->FindNearestNeighbor();
			cout << "Found nearest points with count:" << this->nearestPts.size() << endl;
			/*sort the nearestPts wrt the first index in ascending order because parallel loop
			can shuffle the indices */
			sort(this->nearestPts.begin(), this->nearestPts.end());
			/*step 3 : Data transformation - from R and T matrix to tranform data set close to model set*/
			this->CalculateTransformationMatrix(true);

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
			cout << "Difference in error: " << abs(oldError - this->error) << endl;
			oldError = this->error;
			/*calculate the error*/
			this->error = this->CalculateDistanceError();
			nearestPts.clear();
			iterations++;
			step = true;
		}
		if (abs(oldError - this->error) <= this->minThreshold) {
			cout << "*********************************************************\n";
			cout << "Converged with error:" << abs(oldError - this->error) << endl;
			step = false;
			break;
		}
		if (iterations > this->maxIterations) {
			cout << "*********************************************************\n";
			cout << "Max iterations over, not converged" << endl;
			step = false;
			break;
		}
	}
	/*end the timer*/
	auto finish = std::chrono::high_resolution_clock::now();
	/*calculate the time needed for the whole process*/
	std::chrono::duration<double> elapsed = (finish - start);
	cout << "Time taken = " << elapsed.count()/60. << " minutes" << endl;
	/*save the point cloud*/
	cout << "Saving the registered point cloud....." << endl;
	string fileName = "RegisteredDataICP.xyz";
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
		modelPt.at<double>(0, 0) = modelPCL.pts[i].dataPt.x;
		modelPt.at<double>(1, 0) = modelPCL.pts[i].dataPt.y;
		modelPt.at<double>(2, 0) = modelPCL.pts[i].dataPt.z;
		error += norm((this->Rotation * dataPt + this->Translation) - modelPt);
	}
	error /= dataPCL.pts.size();
	cout << "error between transformed point and the closest point:" << error << endl;
	return error;
}

Mat CloudRegistration::CalcualateICPCovarianceMtx(int length, vector<Point3d>& centerPCL, vector<Point3d>& centerMCL) {
	Mat covariance = Mat::zeros(3, 3, CV_64F);
	double sumXX = 0., sumXY = 0., sumXZ = 0.;
	double sumYX = 0., sumYY = 0., sumYZ = 0.;
	double sumZX = 0., sumZY = 0., sumZZ = 0.;
	for (int i = 0; i < length; i++) {
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
	covariance.at<double>(0, 0) = sumXX / length;
	covariance.at<double>(0, 1) = sumXY / length;
	covariance.at<double>(0, 2) = sumXZ / length;
	covariance.at<double>(1, 0) = sumYX / length;
	covariance.at<double>(1, 1) = sumYY / length;
	covariance.at<double>(1, 2) = sumYZ / length;
	covariance.at<double>(2, 0) = sumZX / length;
	covariance.at<double>(2, 1) = sumZY / length;
	covariance.at<double>(2, 2) = sumZZ / length;

	return covariance;
}

void CloudRegistration::CalculateTransformationMatrix(bool isICP) {
	Point3d pclCOM, mclCOM;
	Mat R, T;
	R = Mat::zeros(3, 3, CV_64F);
	T = Mat::zeros(3, 1, CV_64F);
	vector<Point3d> centerPCL, centerMCL;
	int vectorSize = 0;
	Mat covariance;
	if (isICP) {
		/*step 1 : find center of mass of both the datasets*/
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
		/*for data point cloud*/
		for (int i = 0; i < dataPCL.pts.size(); i++) {
			Point3d pt;
			pt = dataPCL.pts[i].dataPt - pclCOM;
			centerPCL.emplace_back(pt);
		}
		/*for nearest model point cloud*/
		for (int i = 0; i < modelPCL.pts.size(); i++) {
			Point3d pt;
			pt = modelPCL.pts[i].dataPt - mclCOM;
			centerMCL.emplace_back(pt);
		}
		/*step 3 : calculate covariance matrix*/
		vectorSize = nearestPts.size();
		covariance = this->CalcualateICPCovarianceMtx(vectorSize, centerPCL, centerMCL);
	}
	else {
		/*step 1 : find center of mass of both the datasets*/
		/*for data point cloud*/
		for (int i = 0; i < squareDist.size(); i++) {
			pclCOM += dataPCL.pts[trimmedPts[i].second].dataPt;
		}
		pclCOM = pclCOM * (1.0 / dataPCL.pts.size());
		/*for nearest model point clouds*/
		for (int i = 0; i < squareDist.size(); i++) {
			mclCOM += modelPCL.pts[squareDist[i].second].dataPt;
		}
		mclCOM = mclCOM * (1.0 / modelPCL.pts.size());
		cout << "pclCOM: " << pclCOM << " mclCOM: " << mclCOM << endl;
		/*step 2 : center the point cloud as per the center of mass calculated*/
		/*for trimmed data point cloud*/
		for (int i = 0; i < this->squareDist.size(); i++) {
			Point3d pt;
			pt = dataPCL.pts[trimmedPts[i].second].dataPt - pclCOM;
			centerPCL.emplace_back(pt);
		}
		/*for trimmed model point cloud*/
		for (int i = 0; i < this->squareDist.size(); i++) {
			Point3d pt;
			pt = modelPCL.pts[squareDist[i].second].dataPt - mclCOM;
			centerMCL.emplace_back(pt);
		}
		/*step 3 : calculate covariance matrix*/
		vectorSize = squareDist.size();
		covariance = this->CalcualateTrICPCovarianceMtx(vectorSize, centerPCL, centerMCL);
	}

#if SVD_REGISTRATION

	/*SVD registrayion method*/
	/*step 4: eigenvalues and eigenvectors from covariance matrix*/
	Mat w, u, vt;
	SVD::compute(covariance, w, u, vt, 0);

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

	this->nearestPts.clear();
	this->squareDist.clear();
	this->trimmedPts.clear();
//#pragma omp parallel for
	for (int i = 0; i < dataPCL.pts.size(); i++) {
			//cout << i << " ";
			//if ((this->nearestPts.size() > 0) && (this->nearestPts.size() % 10000) == 0)
			//	cout << this->nearestPts.size() << " neighbors found" << endl;
			//Find minimum distance from points in the model set
			double  query_pt[3] = { dataPCL.pts[i].dataPt.x, dataPCL.pts[i].dataPt.y, dataPCL.pts[i].dataPt.z };
			myKDTree index(3, modelPCL, KDTreeSingleIndexAdaptorParams(10));
			index.buildIndex();
			size_t ret_index;
			double out_dist_sqr;
			{
				// do a knn search
				const size_t num_results = 1;
				nanoflann::KNNResultSet<double> resultSet(num_results);
				resultSet.init(&ret_index, &out_dist_sqr);
				index.findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

				//std::cout << "knnSearch(nn=" << num_results << "): \n";
				//std::cout << "i="  << i <<  " ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;
			}
			pair<uint16_t, size_t> nearComb;
			pair<double, uint16_t> sqDist, trim;
			nearComb.first = i;
			nearComb.second = ret_index;

			sqDist.first = out_dist_sqr;
			sqDist.second = ret_index;
			trim.first = out_dist_sqr;
			trim.second = i;
			
			/*nearest points needed for ICP*/
			this->nearestPts.push_back(nearComb);
			/*square distances needed for TrICP*/
			this->squareDist.push_back(sqDist);
			this->trimmedPts.push_back(trim);
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
