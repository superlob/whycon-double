#include<iostream>
#include<stdio.h>
#include<math.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "double_camera_sub/doublecamera.h"

using namespace Eigen;
using namespace std;


#define numposition 56 //�����


double matA[numposition][9];

double get_deficit(Matrix3d R, double t[3], double x1[][3], double x2[][3])
{
	double d = 0;
	for (int i = 0; i <= numposition / 5; i++)
	{
		d += fabs(R(0, 0) * x1[i][0] + R(0, 1) * x1[i][1] + R(0, 2) * x1[i][2] + t[0] - x2[i][0]) +
			      fabs(R(1, 0) * x1[i][0] + R(1, 1) * x1[i][1] + R(1, 2) * x1[i][2] + t[1] - x2[i][1]) +
			      fabs(R(2, 0) * x1[i][0] + R(2, 1) * x1[i][1] + R(2, 2) * x1[i][2] + t[2] - x2[i][2]);
	}
	return d;
}

doublecamera::doublecamera()
{
	matrA = new double* [numposition];
	matrE = new double* [3];
}


doublecamera::~doublecamera()
{
	delete[] matrA;
	delete[] matrE;
}


void doublecamera::get_T(double x1[][3], double x2[][3], Matrix3d R, double* t)
{

	
	for (int i = 0; i <= numposition/5 ; i++)
	{
		t[0] += -R(0, 0) * x1[i][0] - R(0, 1) * x1[i][1] - R(0, 2) * x1[i][2] + x2[i][0];
		t[1] += -R(1, 0) * x1[i][0] - R(1, 1) * x1[i][1] - R(1, 2) * x1[i][2] + x2[i][1];
		t[2] += -R(2, 0) * x1[i][0] - R(2, 1) * x1[i][1] - R(2, 2) * x1[i][2] + x2[i][2];
	}

	t[0] = t[0] / (numposition / 5 + 1), t[1] = t[1] / (numposition / 5 + 1), t[2] = t[2] / (numposition / 5 + 1);

}

void doublecamera::get_row(double a[], double b[], int i)
{

	a[0] = a[0] / a[2];
	a[1] = a[1] / a[2];
	a[2] = 1.0;
	b[0] = b[0] / b[2];
	b[1] = b[1] / b[2];
	b[2] = 1.0;


	matA[i][0] = a[0] * b[0];
	matA[i][1] = a[0] * b[1];
	matA[i][2] = a[0];
	matA[i][3] = a[1] * b[0];
	matA[i][4] = a[1] * b[1];
	matA[i][5] = a[1];
	matA[i][6] = b[0];
	matA[i][7] = b[1];
	matA[i][8] = 1.0;
}

void doublecamera::get_E()
{
	double At[9][numposition], a[9][9] = { 0 };
	int i, j, p, n, row = 3, column = 3;
	double min;

	for (i = 0; i < 9; i++)//求A的转置
		for (j = 0; j < numposition; j++)
			At[i][j] = this->matrA[j][i];
	for (i = 0; i <= 8; i++)//求矩阵At乘A
	{
		for (j = 0; j <= 8; j++)
		{
			for (p = 0; p <numposition; p++)
			{
				a[i][j] += At[i][p] * this->matrA[p][j];
			}
		}
	}
	MatrixXd aa(9, 9);//将矩阵复制到库中参与后续计算
	for (i = 0; i < 9; i++)
	{
		for (j = 0; j < 9; j++)
			aa(i, j) = a[i][j];
	}
	EigenSolver<MatrixXd> es(aa);
	MatrixXd D = es.pseudoEigenvalueMatrix();//求矩阵a特征值和特征向量
	MatrixXd V = es.pseudoEigenvectors();
	min = D(0, 0);
	for (i = 0, j = 0; i < 9, j < 9; i++, j++)//找到最小特征值
	{
		if (min > D(i, j))
		{
			min = D(i, j);
			n = i;
		}
	}
	for (i = 0; i < 3; i++)//最小特征值对应特征向量
	{
		matrE[0][i] = V(i * 3, n);
		matrE[1][i] = V(i * 3 + 1, n);
		matrE[2][i] = V(i * 3 + 2, n);
	}


}

void doublecamera::get_UV()
{
	Matrix3d  A;
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			A(i, j) = this->matrE[i][j];

	JacobiSVD<Eigen::MatrixXd> svd(A, ComputeThinU | ComputeThinV);
	matrV = svd.matrixV(), matrU = svd.matrixU();
	// matrU << 3.62482e-014, 3.70408e-014, -1, 0.707107, 0.707107, 5.18231e-014, -0.707107, 0.707107, 5.60505e-016;
	 //matrV << 0.707107, 0.707107, -1.89307e-013, -0.707107, 0.707107, - 1.58239e-013, 2.1968e-014, 2.45752e-013, 1;

}


Matrix3d doublecamera::get_R(Matrix3d W)
{

	Matrix3d temp1, temp2, temp3, temp4, temp5, temp6;

	temp1 = matrV.transpose();
	temp2 = matrU.transpose();
	temp3 = W.inverse();
	temp4 = temp2.inverse();
	temp5 = temp4 * temp3;
	temp6 = temp5 * temp1;
	return temp6;
}

void doublecamera::get_RT(double a[][3], double b[][3])
{
	Matrix3d R1, R2, R3, R4, W1, W2, W3, W4;
	W1 << 0, 1.0, 0, -1.0, 0, 0, 0, 0, 1.0;
	W2 << 0, -1.0, 0, 1.0, 0, 0, 0, 0, 1.0;
	W3 << 0, 1.0, 0, -1.0, 0, 0, 0, 0, -1.0;
	W4 << 0, -1.0, 0, 1.0, 0, 0, 0, 0, -1.0;
	double E[3][3] = { 0,0,0,0.707,0,0,0,0.707,0 }, T[4][3] = { 0 };
	double** t1 = new double* [4];
	double x1[numposition / 5 + 1][3] = { 0 }, x2[numposition / 5 + 1][3] = { 0 };
    double x3[numposition / 5 + 1][3] = { 0 }, x4[numposition / 5 + 1][3] = { 0 };

	for (int i = 0; i < 4; i++)
	{
		t1[i] = T[i];
	}

	int j = 0;
	for (int i = 0; i <= numposition / 5; i++)
	{
		x1[i][0] = a[j][0], x1[i][1] = a[j][1], x1[i][2] = a[j][2];
		x2[i][0] = b[j][0], x2[i][1] = b[j][1], x2[i][2] = b[j][2];
		j = (i + 1) * 5 - 1;
	}

	int k = 0;
	for (int i = 0; i <= numposition / 5; i++)
	{
		x3[i][0] = a[k][0], x3[i][1] = a[k][1], x3[i][2] = a[k][2];
		x4[i][0] = b[k][0], x4[i][1] = b[k][1], x4[i][2] = b[k][2];
		k = (i + 1) * 5 - 2;
	}

	for (int i = 0; i < 3; i++)
	{
		this->matrE[i] = E[i];
	}

	for (int i = 0; i < numposition; i++)
	{
		this->get_row(a[i], b[i], i);
	}

	for (int i = 0; i < numposition; i++)
	{
		this->matrA[i] = matA[i];
	}

	this->get_E();
	this->get_UV();

	R1 = get_R(W1);
	R2 = get_R(W2);
	R3 = get_R(W3);
	R4 = get_R(W4);

	get_T(x3, x4, R1, t1[0]);
	get_T(x3, x4, R2, t1[1]);
	get_T(x3, x4, R3, t1[2]);
	get_T(x3, x4, R4, t1[3]);


	double s[4] = { 0 }, min;

	s[0] = get_deficit(R1, t1[0], x1, x2);
	s[1] = get_deficit(R2, t1[1], x1, x2);
	s[2] = get_deficit(R3, t1[2], x1, x2);
	s[3] = get_deficit(R4, t1[3], x1, x2);

	min = *min_element(s, s + 4);
	if (min == s[0])
	{
		this->R = R1;
		for (int j = 0; j <= 2; j++)
		{
			this->t[j] = t1[0][j];
		}
	}
	else if (min == s[1])
	{
		this->R = R2;
		for (int j = 0; j <= 2; j++)
		{
			this->t[j] = t1[1][j];
		}
	}
	else if (min == s[2])
	{
		this->R = R3;
		for (int j = 0; j <= 2; j++)
		{
			this->t[j] = t1[2][j];
		}
	}
	else if (min == s[3])
	{
		this->R = R4;
		for (int j = 0; j <= 2; j++)
		{
			this->t[j] = t1[3][j];
		}
	}
}
