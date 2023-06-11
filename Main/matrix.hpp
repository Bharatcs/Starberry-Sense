/*
 * matrix.c
 *
 *  Created on: Dec 22, 2017
 *      Author: mayuresh
 *  Modified on 29-4-20
 *     Author: bharat
 */

#include "math.h"



void mat_add(float a[3][3],float b[3][3], float c[3][3]);
void mat_sub(float a[3][3], float b[3][3], float c[3][3]);
void vec_mul_1(float a[3], float b[3], float c[3][3]); 	// Multiplication to get matrix
float vec_mul_2(float a[3], float b[3]);				// Multiplication to get scalar
void mat_vec_mul(float a[3][3], float b[3], float c[3]);
void transpose(float a[3][3], float b[3][3]);
float trace(float a[3][3]);
void inverse(float a[3][3], float b[3][3]);
float det(float a[3][3]);
void cofactor(float a[3][3], float b[3][3]);
