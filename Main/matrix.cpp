/*
 * matrix.c
 *
 *  Created on: Dec 22, 2017
 *      Author: mayuresh
 *  Modified on 29-4-20
 *     Author: bharat
 */

#include "matrix.hpp"


void mat_add(float a[3][3], float b[3][3], float c[3][3])
{

	for(int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			c[i][j] = a[i][j] + b[i][j];
		}
	}
}

void mat_sub(float a[3][3], float b[3][3], float c[3][3])
{

	for (int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			c[i][j] = a[i][j] - b[i][j];
		}
	}
}

void vec_mul_1(float a[3], float b[3], float c[3][3])	// Multiplication to get a matrix
{

	for (int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			c[i][j] = a[i]*b[j];
		}
	}
}

float vec_mul_2(float a[3], float b[3])					// Multiplication to get a scalar
{
	float val=0.0;

	for (int i=0;i<3;i++)
	{
		val = val + a[i]*b[i];
	}
	return val;
}

void mat_vec_mul(float a[3][3], float b[3], float c[3])
{

	for (int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			c[i] = c[i] + a[i][j]*b[j];
		}
	}
}

void transpose(float a[3][3], float b[3][3])
{

	for (int i=0;i<3;i++)
	{
		for (int j=0;j<3;j++)
		{
			b[i][j] = a[j][i];
		}
	}
}

float trace(float a[3][3])
{
	float val=0.0;
	for (int i=0;i<3;i++)
	{
		val = val + a[i][i];
	}

	return val;
}

void inverse(float A[3][3], float result[3][3])
{
	float determinant =    A[0][0]*(A[1][1]*A[2][2]-A[2][1]*A[1][2])
	                        -A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
	                        +A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
	float invdet = 1/determinant;
	result[0][0] =  (A[1][1]*A[2][2]-A[2][1]*A[1][2])*invdet;
	result[1][0] = -(A[0][1]*A[2][2]-A[0][2]*A[2][1])*invdet;
	result[2][0] =  (A[0][1]*A[1][2]-A[0][2]*A[1][1])*invdet;
	result[0][1] = -(A[1][0]*A[2][2]-A[1][2]*A[2][0])*invdet;
	result[1][1] =  (A[0][0]*A[2][2]-A[0][2]*A[2][0])*invdet;
	result[2][1] = -(A[0][0]*A[1][2]-A[1][0]*A[0][2])*invdet;
	result[0][2] =  (A[1][0]*A[2][1]-A[2][0]*A[1][1])*invdet;
	result[1][2] = -(A[0][0]*A[2][1]-A[2][0]*A[0][1])*invdet;
	result[2][2] =  (A[0][0]*A[1][1]-A[1][0]*A[0][1])*invdet;
}

float det(float a[3][3])
{

	return a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) - a[0][1]*(a[1][0]*a[2][2] - a[1][2]*a[2][0]) + a[0][2]*(a[1][0]*a[2][1] - a[1][1]*a[2][0]);

}

void cofactor(float a[3][3], float b[3][3])
{
	b[0][0] = a[1][1]*a[2][2] - a[1][2]*a[2][1];
	b[0][1] = a[1][2]*a[2][0] - a[1][0]*a[2][2];
	b[0][2] = a[1][0]*a[2][1] - a[1][1]*a[2][0];
	b[1][0] = a[0][2]*a[2][1] - a[0][1]*a[2][2];
	b[1][1] = a[0][0]*a[2][2] - a[0][2]*a[2][0];
	b[1][2] = a[0][1]*a[2][0] - a[0][0]*a[2][1];
	b[2][0] = a[0][1]*a[1][2] - a[0][2]*a[1][1];
	b[2][1] = a[0][2]*a[1][0] - a[0][0]*a[1][2];
	b[2][2] = a[0][0]*a[1][1] - a[0][1]*a[1][0];
}
