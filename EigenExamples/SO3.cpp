#include "SO3.h"

using namespace Geometry;

int main() {

	/*********************************SO3 Geometry********************************/
	/*std::cout << "*********************Lie Geometry*******************" << std::endl;
	std::cout << SO3::Log(Mat3::Identity()) << std::endl;
	std::cout <<
		SO3::Exp(SO3::Log(Mat3::Identity())) << std::endl;*/

	srand(time(0));
	auto quat = Eigen::Quaterniond::UnitRandom();
	auto M1 = quat.toRotationMatrix();
	std::cout << M1 << std::endl;
	std::cout << SO3::Log(M1) << std::endl;
	std::cout << SO3::Exp(SO3::Log(M1)) << std::endl;

	/*Box minus*/
	auto quat0 = Eigen::Quaterniond::UnitRandom();
	auto quat1 = Eigen::Quaterniond::UnitRandom();
	auto X = quat0.toRotationMatrix();
	auto Y = quat1.toRotationMatrix();
	std::cout << "X = \n" << X << std::endl;
	std::cout << "Y = \n" << Y << std::endl;
	std::cout << SO3::boxminus(X, Y) << std::endl;

	return 0;
}