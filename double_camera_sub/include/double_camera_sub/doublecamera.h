#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace Eigen;

class doublecamera
{
public:

	doublecamera();

	~doublecamera();

	void get_T(double x1[][3], double x2[][3], Matrix3d R, double* t);

	void get_row(double a[], double b[], int i);

	void get_E();

	void get_UV();

	Matrix3d get_R(Matrix3d W);

	void get_RT(double a1[][3], double b1[][3]);

	double** matrA, ** matrE;
	Matrix3d matrU, matrV;
	Matrix3d R;
	double t[3];
};