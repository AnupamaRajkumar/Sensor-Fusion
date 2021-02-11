#include "Utils.h"


/**************************************************************************
Function Name    : LoadData
Parameters		 : 3D point cloud of format x, y, z, R, G, B
Returns			 : void
Function		 : Loads the point cloud from xyz/txt file
Author			 : Anupama Rajkumar
****************************************************************************/
void Utils::LoadData(std::vector<PointXYZRGB>& points, char* fileName) {
	std::ifstream datafile(fileName);
	std::string line;
	int lineCounter = 0;
	if (datafile.is_open()) {
		while (!datafile.eof()) {
			cv::Point3d point;
			std::vector<std::string> coords;
			std::string word = " ";
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
			//cout << word << endl;
			coords.push_back(word);
			size_t sz;
			point.x = stod(coords[0], &sz);
			point.y = stod(coords[1], &sz);
			point.z = stod(coords[2], &sz);
			/*RGB in xyz file, BGR in OpenCV, hence order is flipped*/
			PointXYZRGB aP = { point, stoi(coords[5], &sz) , stoi(coords[4], &sz) , stoi(coords[3], &sz), 0 };
			points.emplace_back(aP);
		}
	}
	else {
		std::cerr << "Cannot open file" << std::endl;
		exit(-1);
	}
}

/**************************************************************************
Function Name    : WriteDataPoints
Parameters		 : 3D point cloud of format x, y, z, R, G, B
Returns			 : void
Function		 : Writes the clustered point cloud to an xyz file
Author			 : Anupama Rajkumar
****************************************************************************/
void Utils::WriteDataPoints(std::vector<PointXYZRGB>& points, std::string fileName) {
	std::ofstream dataFile;
	dataFile.open(fileName);
	std::cout << "Writing the points into file" << std::endl;
	for (int i = 0; i < points.size(); i++) {
		dataFile << points[i].point.x << " " << points[i].point.y << " " << points[i].point.z << " " <<
			points[i].red << " " << points[i].green << " " << points[i].blue << " " << std::endl;
	}
	dataFile.close();
}

/**************************************************************************
Function Name    : PointColorAssignment
Parameters		 : void
Returns			 : red, green and blue values to be assigned to a point
Function		 : Generates random color values to be assigned to points
Author			 : Anupama Rajkumar
****************************************************************************/
cv::Point3d Utils::PointColorAssignment(){
	cv::Point3d colorVal;
	int red, blue, green;

	std::random_device rd;
	std::mt19937 gen(rd());							// seed the generator
	std::uniform_int_distribution<> distr(0, 255); // define the range
	red = distr(gen);
	blue = distr(gen);
	green = distr(gen);

	colorVal.x = red;
	colorVal.y = green;
	colorVal.z = blue;
	//std::cout << colorVal.x << " " << colorVal.y << " " << colorVal.z << std::endl;
	return colorVal;
}

/**************************************************************************
Function Name    : Distance
Parameters		 : Points between which the distance needs to be determined
Returns			 : Distance (sqaure) between the two points
Function		 : Calculates the distance between two points
Author			 : Anupama Rajkumar
****************************************************************************/
double Utils::Distance(cv::Point3d x, cv::Point3d y) {
	double dist;
	dist = (x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y) + (x.z - y.z)*(x.z - y.z);
	return dist;
}
