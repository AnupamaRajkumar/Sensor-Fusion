#pragma once
#ifndef __SO3__
#define __SO3__


#define _USE_MATH_DEFINES
#include <iostream>
#include <Eigen/Dense>
#include <time.h>

namespace Geometry {

	using Mat3 = Eigen::Matrix3d;
	using Vec3 = Eigen::Vector3d;

	namespace SO3 {
		// TODO: Exp, Log, boxplus, boxminus, interpolation
		inline
			Mat3 Exp(const Vec3& w) {
			Mat3 w_hat;
			w_hat <<
				0, -w.z(), w.y(),
				w.z(), 0, -w.x(),
				-w.y(), w.x(), 0;

			//eg w.cross(some_other_vector)== w_hat * some_other_vector;

			/*const double phi = w.norm(); //l2 norm
			//divide by 0 error as phi->0
			return Mat3::Identity() +
				w_hat / phi * sin(phi) +
				w_hat * w_hat / (phi * phi) * (1 - cos(phi));*/


			/*we do this so that we can take care of the case when phi --> 0*/
			const double phi2 = w.squaredNorm();
			if (phi2 > std::numeric_limits<float>::epsilon()) {
				const double phi = std::sqrt(phi2);
				return Mat3::Identity() +
					w_hat / phi * sin(phi) +
					w_hat * w_hat / (phi * phi) * (1 - cos(phi));
			}
			else {
				return Mat3::Identity() + w_hat;
			}
		}

		inline
			Vec3 Log(const Mat3& R) {

			// question: how to improve? (homework) +1.0p
			const double phi = std::acos((R.trace() - 1) / 2);

			if (phi > std::numeric_limits<float>::epsilon()) {
				Vec3 w_normalized = 1 / (2 * sin(phi)) * Vec3(
					R(2, 1) - R(1, 2),
					R(0, 2) - R(2, 0),
					R(1, 0) - R(0, 1)
				);

				return phi * w_normalized;
			}
			else {
				return (1.0 / 2.0) * Vec3(
					R(2, 1) - R(1, 2),
					R(0, 2) - R(2, 0),
					R(1, 0) - R(0, 1)
				);
			}
		}

		inline
			Vec3 boxminus(const Mat3& X, const Mat3& Y) {
			return(Log(X * Y.inverse()));
		}

		inline
			Mat3 boxplus(const Mat3& X, const Vec3& u) {
			return(Exp(u) * X);
		}

		inline
			Mat3 interpolate(const Mat3& X, const Mat3& Y, const double& t) {
			// t == 0 -> Y
			// t == 1 -> X
			// t \in (0,1) -> .....
		}

		/*interpolate
		t == 0 -> Y
		t == 1 -> X
		t in (0,1) -> ....
		*/

	}

}

#endif // !__SO3__