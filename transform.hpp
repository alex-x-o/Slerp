#ifndef _TRANSFORM_
#define _TRANSFORM_

#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>
#include <Eigen/Geometry>
 
using namespace Eigen;

typedef EulerSystem<EULER_X, EULER_Y, EULER_Z> MySystem;
typedef EulerAngles<double, MySystem> MyAngles;

Matrix3d Euler2A(const double& phi, const double& theta, const double& psi);
std::pair<RowVector3d, double> AxisAngle(const Matrix3d& A);
Matrix3d Rodrigez(const Vector3d& p, const double& phi);
std::tuple<double, double, double> A2Euler(const Matrix3d& A);
Quaterniond AxisAngle2Q(const Vector3d& p, const double& phi);
std::pair<RowVector3d, double> Q2AxisAngle(const Quaterniond& q);
Quaterniond slerp(Quaterniond& q1, Quaterniond& q2, const double tm, const double t);

#endif