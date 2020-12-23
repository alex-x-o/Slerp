#include "transform.hpp"

Matrix3d Euler2A(const double& phi, const double& theta, const double& psi)
{
    MyAngles vehicleAngles(phi, theta, psi);

    // I
    Matrix3d result;
    result(0, 0) = cos(theta) * cos(psi);
    result(0, 1) = cos(psi) * sin(theta) * sin(phi) - cos(phi) * sin(psi);
    result(0, 2) = cos(phi) * cos(psi) * sin(theta) + sin(phi) * sin(psi);
    result(1, 0) = cos(theta) * sin(psi);
    result(1, 1) = cos(psi) * cos(phi) + sin(theta) * sin(phi) * sin(psi);
    result(1, 2) = cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi);
    result(2, 0) = -sin(theta);
    result(2, 1) = cos(theta) * sin(phi);
    result(2, 2) = cos(theta) * cos(phi);

    // II
    Matrix3d R_phi;
    Matrix3d R_theta;
    Matrix3d R_psi;

    R_phi << 1, 0, 0,
             0, cos(phi), -sin(phi),
             0, sin(phi), cos(phi);
        
    R_theta << cos(theta), 0, sin(theta),
               0, 1, 0,
               -sin(theta), 0, cos(theta);

    R_psi << cos(psi), -sin(psi), 0,
             sin(psi), cos(psi), 0,
             0, 0, 1;

    result = R_psi * R_theta * R_phi;

    // III
    result = AngleAxisd(psi, Vector3d::UnitZ())
             * AngleAxisd(theta, Vector3d::UnitY())  
             * AngleAxisd(phi, Vector3d::UnitX());
    
    return result;
}

bool isOrthogonal(const Matrix3d& A)
{
    return (A.transpose() * A).isIdentity(1000) ? true : false;    
}

bool isDetOne(const Matrix3d& A)
{
    return (std::abs(A.determinant() - 1) <= std::abs(A.determinant() + 1) * 8 * std::numeric_limits<double>::epsilon()) ? true : false;
}

std::pair<RowVector3d, double> AxisAngle(const Matrix3d& A)
{
    if(!isOrthogonal(A))
    {
            std::cout << "Not an orthogonal matrix!" << std::endl;
            exit(1);
    }
    if(!isDetOne(A))
    {
            std::cout << "Determinant of a matrix is not 1!" << std::endl;
            exit(1);
    }
    // finding eigenvector of matrix A for eigenvalue 1
    Matrix3d Ap = A - Matrix3d::Identity();
    Vector3d u = Ap.row(0);
    Vector3d v = Ap.row(1);
    Vector3d p = u.cross(v);
    if(p == Vector3d::Zero())
    {
        Vector3d w = Ap.row(2);
        p = u.cross(w);
    }
    p.normalize();
    // we are choosing arbitrary vector normal to p - in our case vector u
    u.normalize();
    Vector3d up = A * u;
    up.normalize();
    double phi = acos(u.dot(up));
    double tripleProd = u.dot(up.cross(p));
    if(tripleProd < 0)
    {
        p = -p;
    }
    return std::make_pair(p, phi);
}

Matrix3d Rodrigez(const Vector3d& p, const double& phi)
{
    Matrix3d ppt = p * p.transpose();
    Matrix3d mat = Matrix3d::Identity() - ppt;
    Matrix3d px;
    px  <<  0, -p(2), p(1),
            p(2), 0, -p(0),
            -p(1), p(0), 0;
        
    return ppt + cos(phi)*mat + sin(phi)*px;
}

std::tuple<double, double, double> A2Euler(const Matrix3d& A)
{
    if(!isOrthogonal(A))
    {
        std::cout << "Not an orthogonal matrix!" << std::endl;
        exit(1);
    }
    if(!isDetOne(A))
    {
        std::cout << "Determinant of a matrix is not 1!" << std::endl;
            exit(1);
    }
    double psi, theta, phi;
    if(A(2, 0) < 1)
    {
        if(A(2,0) > -1)
        {
            psi = atan2(A(1, 0), A(0, 0));
            theta = asin(-A(2, 0));
            phi = atan2(A(2, 1), A(2, 2));
        }
        else
        {
            psi = atan2(-A(0, 1), A(1, 1));
            theta = M_PI_2;
            phi = 0;
        }
    }
    else
    {
        psi = atan2(-A(0, 1), A(1, 1));
        theta = -M_PI_2;
        phi = 0;
    }

    return std::make_tuple(phi, theta, psi);
}

Quaterniond AxisAngle2Q(const Vector3d& p, const double& phi)
{
    double w = cos(phi/2);
    Vector3d r = p.normalized();
    double x = sin(phi/2)*r(0);
    double y = sin(phi/2)*r(1);
    double z = sin(phi/2)*r(2);

    return Quaterniond(w, x, y, z);
}

std::pair<RowVector3d, double> Q2AxisAngle(const Quaterniond& q)
{
    Quaterniond r = q.normalized();
    double phi = 2*acos(r.w());
    double w = r.w();
    Vector3d p;
    if(w == 1 || w == -1)
    {
        p(0) = 1;
        p(1) = 0;
        p(2) = 0;
    }
    else 
    {
        p = q.vec().normalized();
    }
    return std::make_pair(p, phi);
}

Quaterniond slerp(Quaterniond& q1, Quaterniond& q2, const double tm, const double t)
{
	double dot = q1.dot(q2);

	if(dot < 0.0)
	{
		q1.coeffs() = -q1.coeffs();
		dot = -dot;
	}
	if(dot > 0.95)
	{
		double f = 1 - t / tm;
		double s = t / tm;
		return Quaterniond(f * q1.coeffs() + s * q2.coeffs());
	}
	double theta0 = acos(dot);
	double theta = theta0 * t / tm;
	double sinTheta0 = sin(theta0);
	double sinTheta = sin(theta);

	double f = sin(theta0 - theta) / sinTheta0;
	double s = sinTheta / sinTheta0;

	return Quaterniond(f * q1.coeffs() + s * q2.coeffs());
}
