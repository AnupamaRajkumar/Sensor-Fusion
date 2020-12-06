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

			/*Typical Rodrigues Formula implemetation*/
			/*const double phi = w.norm(); //l2 norm
			//divide by 0 error as phi->0
			return Mat3::Identity() +
				w_hat / phi * sin(phi) +
				w_hat * w_hat / (phi * phi) * (1 - cos(phi));*/


			/*we do this so that we can take care of the case when phi --> 0*/
			const double phi2 = w.squaredNorm();
			if (phi2 > std::numeric_limits<float>::epsilon()) {
				const double phi = std::sqrt(phi2);
				/*Rodrigues Formula for solving so(3)*/
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
			//const double phi = std::acos((R.trace() - 1) / 2);
			/*Problems with inverse cos function
			- if angle is greater than pi, it is difficult to decide which quadrant the value of phi lies in
			- use atan2 for better performance as it takes values of x and y 
			- around 0 and 1, precision of acos is low (~10^-6..7)
			- numerical precision of atan2 is higher than acos (~10^12 or higher) and hence margin of error is much lower
			*/
			const Vec3 w = Vec3(
				R(2, 1) - R(1, 2),
				R(0, 2) - R(2, 0),
				R(1, 0) - R(0, 1)
			);

			/* normalising w:
			1. divide by (2 * sin(phi))
			2. divide by w.norm()
			*/

			const double cos_phi = (R.trace() - 1) / 2;
			const double sin_phi = w.norm() / 2.0;  //== sin(phi)
			const double phi = std::atan2(sin_phi, cos_phi);

			if (sin_phi > std::numeric_limits<float>::epsilon() && (phi != 0. || phi != M_PI)) {
				//case 1
				return phi / (2 * sin_phi) * w;
			}
			else if (sin_phi == 0. && phi == 0.)
			{
				/*only works when phi ~ 0 and sin(phi) ~ 0*/
				//case 2
				return (1.0 / 2.0) * w;
			}
			else {
				/*what happens when phi ~pi but sin(phi) ~0 ? [+4 points]*/
				//case 3	
				/*as x -> pi, sin(pi)/pi = cos(pi) = -1*/
				return (-1.0 / 2.0) * w;
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
			Vec3 u = boxminus(Y, X);
			return(boxplus(X, t*u));
		}

		/* https://blog.demofox.org/2015/07/05/the-de-casteljeau-algorithm-for-evaluating-bezier-curves/ */
		inline 
			Mat3 QuadraticInterpolate(const Mat3& X, const Mat3& Y,
									  const Mat3& Z, const double& t) {
			/*quadratic interpolation is linear interpolation between X, Y and
			Y and Z and eventually a linear interpolation between their resp.
			results*/
			Mat3 R1 = interpolate(X, Y, t);
			Mat3 R2 = interpolate(Y, Z, t);
			Mat3 Result = interpolate(R1, R2, t);
			return Result;
		}

		inline 
			Mat3 CubicInterpolate(const Mat3& X, const Mat3& Y,
								  const Mat3& Z, Mat3& W, const double& t) {
			/*cubic interpolation is linear interpolation between results of
			quadratic interpolations between X, Y, Z and Y, Z, W respectively*/
			Mat3 R1 = QuadraticInterpolate(X, Y, Z, t);
			Mat3 R2 = QuadraticInterpolate(Y, Z, W, t);
			Mat3 Result = interpolate(R1, R2, t);
			return Result;
		}

	}
}

#endif // !__SO3__