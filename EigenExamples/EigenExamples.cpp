/* EigenExamples.cpp : Some basic examples and trials with Eigen Library
*/



#include "SO3.h"



#if 0 
int main()
{
	using Mat3 = Eigen::Matrix<double, 3, 3>;
	using Mat34 = Eigen::Matrix<double, 3, 4>;
	//Mat3 = Eigen::Matrix3d
	using Vec3 = Eigen::Vector3d;
	
	Mat3 A;
	//A(row, col); row, col <<< 0, 1, 2
	A = Mat3::Zero();
	A = Mat3::Identity();

	std::cout << A << std::endl;

	Vec3 b;
	b << 
		1.0, 
		2.0, 
		3.0;

	std::cout << A * b << std::endl;

	// notes:
	// A + B
	// b + c

	Vec3 c = b.transpose() * A; // A.transpose() * b
	std::cout << c << std::endl;

	//Solving Equations

	// y = B * x 
	// x = ?
	Vec3 y{ 1, 2, 3 };
	Mat3 B = Mat3::Random();
	std::cout << "B:\n" << B << std::endl;
	std::cout << "y:\n" << y << std::endl;
	
	std::cout << "x = B.inverse() * y\n" <<
				 B.inverse() * y << std::endl;

	auto fullPixLU = B.fullPivLu();
	std::cout << "x = B.inverse() * y\n" <<
		fullPixLU.solve(y) << std::endl;

	//[U, S, V] = svd(B)
	std::cout << "x = B.jacobiSvd.solve(y)\n" <<
		B.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(y) << std::endl;

	/**************************Tasks and trials***************************************/

	//task 1:
	//create a 2x2 rotation matrix,
	//parameterised by an angle phi

	double phi = 45; //degrees
	double convertToDegs =  M_PI/ 180;
	phi = phi * convertToDegs; //convert from radians to degrees
	Eigen::Matrix2d R;
	R <<
		cos(phi), -sin(phi),
		sin(phi), cos(phi);

	std::cout << "R:\n" << R << std::endl;

	//then:
	//-rotate a vector
	using Vec2 = Eigen::Vector2d;
	Vec2 v = Vec2::Identity();
	Vec2 rotateVec = R * v;
	std::cout << "Rotated vector:\n" << rotateVec << std::endl;
	//-validate that it has been rotated by phi
	std::cout << "Rotated angle is:" << atan2(rotateVec[1], rotateVec[0]) / convertToDegs << std::endl;

	//task 2:
	//-3x3 rotation matrix around X
	//- rotate Vec3{1, 0, 0} ..What happens?
	//- rotate Vec3{0, 1, 0} ..What happens?
	//- rotate Vec3{0, 0, 1} ..What happens?
	Eigen::Matrix3d R3;
	R3 <<
		1, 0, 0,
		0, cos(phi), -sin(phi),
		0, sin(phi), cos(phi);

	using Vec3 = Eigen::Vector3d;
	Vec3 v3X{ 1, 0, 0 };
	Vec3 rotateVec3 = R3 * v3X;
	std::cout << "{1, 0, 0} Rotated along X:\n" << rotateVec3 << std::endl;
	//-validate that it has been rotated by phi
	std::cout << "Rotated angle is:" << atan2(rotateVec3[1], rotateVec3[0]) / convertToDegs << std::endl;

	Vec3 v3Y{ 0, 1, 0 };
	rotateVec3 = R3 * v3Y;
	std::cout << "{1, 0, 0} Rotated along X:\n" << rotateVec3 << std::endl;
	//-validate that it has been rotated by phi
	std::cout << "Rotated angle is:" << atan2(rotateVec3[2], rotateVec3[1]) / convertToDegs << std::endl;

	Vec3 v3Z{ 0, 0, 1 };
	rotateVec3 = R3 * v3Z;
	std::cout << "{1, 0, 0} Rotated along X:\n" << rotateVec3 << std::endl;
	//-validate that it has been rotated by phi
	std::cout << "Rotated angle is:" << atan2(rotateVec3[2], rotateVec3[1]) / convertToDegs << std::endl;

	/* task 1: Rotation using Eigen APIs*/
	Eigen::Vector2f v2{ 1, 0 };
	Eigen::Rotation2Df t(phi);
	t.toRotationMatrix();
	std::cout << "Rotation Matrix t:\n" << t.matrix() << std::endl;
	Eigen::Vector2f rotatedVec = t * v2;
	std::cout << "Rotated vector:\n" << rotatedVec << std::endl;

	/*Some more decompositions*/
	//1. Calculating eigen values and eigen vectors
	using Mat3f = Eigen::Matrix3f;
	Mat3f eigenMat = Mat3f::Random();
	Eigen::SelfAdjointEigenSolver<Mat3f> eigenSolver(eigenMat);
	if (eigenSolver.info() != Eigen::Success) {
		abort();
	}
	std::cout << "Eigenvalues of eigenMat are:" << eigenSolver.eigenvalues() << std::endl;
	std::cout << "Eigenvectors of eigenMat are:" << eigenSolver.eigenvectors() << std::endl;

	//2. Matrix operations - Inverse, determinant etc
	std::cout << "Determinant of matrix B is:" << B.determinant() << std::endl;
	std::cout << "Inverse of matrix B is:" << B.inverse() << std::endl;
	/*calculate rank of matrix*/
	Eigen::FullPivLU<Mat3> lu_decomp(B);
	std::cout << "Rank of matrix B is:" << lu_decomp.rank() << std::endl;

	return 0;
}
#endif

