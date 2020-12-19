#include "IterativeClosestPoint.h"


/************************************************************
Trimmed iterative closest point (TrICP)
1. Find closest point in the model set such that  |M - S| = min using kd-tree
2. Trimmed squares : sort the distances, select Npo least values and calculate their sum
3. Convergence test : Repeat until any of the stopping conditions is satisfied
4. Motion calculation : Compute optimal motion (R, t) that minimises Sts
5. Data set motion
**************************************************************/

void CloudRegistration::TrimmedICP() {
	/*start the timer*/
	auto start = std::chrono::high_resolution_clock::now();
	int iterations = 1;
	double oldError = 0.0;
	bool step = true;
	while (step) {
		if ((iterations <= this->maxIterations) && 
			!(abs(oldError - this->error) < this->minTrimmedThreshold) &&
			!(CalculateTrimmedMSE(this->overlapParameter) < this->minTrimmedError)) {
			cout << "Iteration number: " << iterations << endl;
			/*step 2 : Data association - for each point in the data set, find the nearest neighbor*/
			this->FindNearestNeighbor();
			cout << "Found nearest points with count:" << this->squareDist.size() << endl;
			/*step 3 : sort the sqaure distances  */
			sort(this->squareDist.begin(), this->squareDist.end());
			sort(this->trimmedPts.begin(), this->trimmedPts.end());
			/*step 4 : Calculate overlap parameter*/
			this->CalculateOverlapParameter();
			/*The trimmed data set contains the elements upto the length caculated by the overlap parameter*/
			/*step 5 : Data transformation - from R and T matrix to tranform data set close to model set*/
			this->CalculateTransformationMatrix(false);
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
			oldError = this->error;
			this->error = this->CalculateTrimmedMSE(this->overlapParameter);
			cout << "Relative change of trimmed mean square error: " << abs(oldError - this->error) << endl;
			cout << "Trimmed MSE: " << this->error << endl;		
			cout << "Distance error is:" << this->CalculateDistanceError() << endl;
			squareDist.clear();
			iterations++;
			step = true;
		}
		if (this->error < this->minTrimmedError) {									/*convergence condition 1*/
			cout << "**************************************************************\n";
			cout << "Converged with trimmed MSE" << this->error << endl;
			step = false;
			break;
		}
		if ((abs(oldError - this->error)) <= this->minTrimmedThreshold) {					/*convergence condition 2*/
			cout << "*********************************************************\n";
			cout << "Converged due to small relative change of trimmed mean square error:" 
				 << abs(oldError - this->error) << endl;
			step = false;
			break;
		}
		if (iterations > this->maxIterations) {										/*convergence condition 3*/
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
	cout << "Time taken = " << elapsed.count() / 60. << " minutes" << endl;
	/*save the point cloud*/
	cout << "Saving the registered point cloud....." << endl;
	string fileName = "RegisteredDataTrICP.xyz";
	this->WriteDataPoints(dataPCL, fileName);
}

Mat CloudRegistration::CalcualateTrICPCovarianceMtx(int length, vector<Point3d>& centerPCL, vector<Point3d>& centerMCL) {
	Mat covariance = Mat::zeros(3, 3, CV_64F);
	length = this->overlapParameter * length;
	double sumXX = 0., sumXY = 0., sumXZ = 0.;
	double sumYX = 0., sumYY = 0., sumYZ = 0.;
	double sumZX = 0., sumZY = 0., sumZZ = 0.;
	for (int i = 0; i < length; i++) {
		sumXX += centerPCL[i].x * centerMCL[i].x;
		sumXY += centerPCL[i].x * centerMCL[i].y;
		sumXZ += centerPCL[i].x * centerMCL[i].z;
		sumYX += centerPCL[i].y * centerMCL[i].x;
		sumYY += centerPCL[i].y * centerMCL[i].y;
		sumYZ += centerPCL[i].y * centerMCL[i].z;
		sumZX += centerPCL[i].z * centerMCL[i].x;
		sumZY += centerPCL[i].z * centerMCL[i].y;
		sumZZ += centerPCL[i].z * centerMCL[i].z;
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
 
/* Golden-section Search  https://en.wikipedia.org/wiki/Golden-section_search */
void CloudRegistration::CalculateOverlapParameter() {
	double minOverlap = 0.4;								//overlap range 0.4 to 1
	double lambda = 2.0;
	double objectiveFun = 0., objectiveFunNext = 0., objectiveFunPrev = 0.;
	bool overlapParamFound = false;
	double overlapIt = 0.05;
	int count = 1;
	while (minOverlap <= 1.0 && !overlapParamFound) {
		//cout << "Iteration number : " << count << endl;
		double trimmedMSEPrev = this->CalculateTrimmedMSE(minOverlap - overlapIt);
		double trimmedMSE = this->CalculateTrimmedMSE(minOverlap);
		double trimmedMSENext = this->CalculateTrimmedMSE(minOverlap + overlapIt);
		objectiveFun = trimmedMSE * (1. / pow(minOverlap, lambda + 1.));
		objectiveFunPrev = trimmedMSEPrev * (1. / pow((minOverlap - overlapIt), lambda + 1.));
		objectiveFunNext = trimmedMSENext * (1. / pow((minOverlap + overlapIt), lambda + 1.));
		//cout << minOverlap << ":" << objectiveFunPrev << "," << objectiveFun << "," << objectiveFunNext << endl;
		if ((objectiveFun < objectiveFunPrev) && (objectiveFunNext > objectiveFun)) {
			this->overlapParameter = minOverlap;
			overlapParamFound = true;
			cout << "overlap parameter found with value : " << objectiveFun 
				 << " for overlap function : " << this->overlapParameter << endl;
			break;
		}
		else {
			minOverlap += overlapIt;
			overlapParamFound = false;
			count++;
		}
		if (minOverlap > 1.0) {
			this->overlapParameter = minOverlap;
			cout << "optimum overlap couldn't be found" << endl;
			break;
		}
	}
}

double CloudRegistration::CalculateTrimmedMSE(double overlap) {
	double trimmedMSE = 0.;
	int length = overlap * this->squareDist.size();
	for (int i = 0; i < length; i++) {
		trimmedMSE += this->squareDist[i].first;
	}
	trimmedMSE /= length;
	return trimmedMSE;
}