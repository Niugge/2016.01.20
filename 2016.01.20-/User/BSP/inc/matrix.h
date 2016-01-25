#ifndef __MATH1_H_
#define __MATH1_H_
#include "stdint.h"
#include "stm32f10x.h"

float determinant(float *p, int n);
float det(float *p, int n, int k, int list[], float sum);
void swap(int *a, int *b);
int inver_order(int list[], int n);
int sgn(int order);
float algebraic_cofactor(float *p, int m, int n, int k);
void adjoint_m(float *m, float *adj, int k);
void inverse_matrix(float*raw, float *inv, int k);
void m_multiply(float *a, float*b, float *c, int ra, int ca, int rb, int cb);

#endif
