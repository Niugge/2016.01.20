#include <stdio.h>
#include "matrix.h"
//--------------------------------------------------------
//功能：求矩阵 n X n 的行列式
//入口参数：矩阵首地址 p；矩阵行数 n
//返回值：矩阵的行列式值
//--------------------------------------------------------
float determinant(float *p, int n)
{
	int t[n];
	int *list =t;
	for (int i = 0; i < n; i++)
		list[i] = i;
	float ret = det(p, n, 0, list, 0.0);
	//delete[] list;
	return ret;
}


float det(float *p, int n, int k, int list[], float sum)
{
	if (k >= n)
	{
		int order = inver_order(list, n);
		float item = (float)sgn(order);
		for (int i = 0; i < n; i++)
		{
			//item *= p[i][list[i]];
			item *= *(p + i * n + list[i]);
		}
		return sum + item;
	}
	else
	{
		for (int i = k; i < n; i++)
		{
			swap(&list[k], &list[i]);
			sum = det(p, n, k + 1, list, sum);
			swap(&list[k], &list[i]);
		}
	}
	return sum;
}

void swap(int *a, int *b)
{
	int m;
	m = *a;
	*a = *b;
	*b = m;
}

//求逆序对的个数
int inver_order(int list[], int n)
{
	int ret = 0;
	for (int i = 1; i < n; i++)
		for (int j = 0; j < i; j++)
			if (list[j] > list[i])
				ret++;
	return ret;
}

int sgn(int order)
{
	return order % 2 ? -1 : 1;
}
//----------------------------------------------------
//功能：求k×k矩阵中元素A(mn)的代数余子式
//入口参数：k×k矩阵首地址；元素A的下标m,n; 矩阵行数 k
//返回值： k×k矩阵中元素A(mn)的代数余子式
//----------------------------------------------------
float algebraic_cofactor(float *p, int m, int n, int k)
{
	int len = (k - 1) * (k - 1);
	float w[len];
	float *cofactor = w;

	int count = 0;
	int raw_len = k * k;
	for (int i = 0; i < raw_len; i++)
		if (i / k != m && i % k != n)
			*(cofactor + count++) = *(p + i);

	float ret = determinant(cofactor, k - 1);
	if ((m + n) % 2)
		ret = -ret;
	//delete[] cofactor;
	return ret;
}


//----------------------------------------------------
//功能：求k×k矩阵的伴随矩阵
//入口参数：m是k×k矩阵首地址；矩阵行数 k；输出参数adj是伴随矩阵的入口地址
//返回值： 无
//----------------------------------------------------
void adjoint_m(float *m, float *adj, int k)
{
	int len = k * k;
	int count = 0;
	for (int i = 0; i < len; i++)
	{
		*(adj + count++) = algebraic_cofactor(m, i % k, i / k, k);
	}
}


//----------------------------------------------------
//功能：求k×k矩阵的逆矩阵
//入口参数：m是k×k矩阵首地址；矩阵行数 k；输出参数inv是逆矩阵的入口地址
//返回值： 无
//----------------------------------------------------
void inverse_matrix(float *raw, float *inv, int k)
{
	float det = determinant(raw, k); //求行列式
	//if (det == 0)
	//{
	//	cout << "矩阵不可逆" << endl;
	//	return;
	//}
	adjoint_m(raw, inv, k); //求伴随矩阵
	int len = k * k;
	for (int i = 0; i < len; i++)
		*(inv + i) /= det;
}


//--------------------------------------------------------
//功能：求矩阵a和b的相乘结果
//入口参数：矩阵首地址 a和b；矩阵a行数ra和列数rc；矩阵b的行数rb和列数cb
//返回值：矩阵a和b的相乘结果
//--------------------------------------------------------
void m_multiply(float *a, float *b, float *c, int ra, int ca, int rb, int cb)
{
	//if (ca != rb)
	//{
	//	cout << "矩阵不可乘" << endl;
	//	return NULL;
	//}

	float *ret = c;
	//if (NULL == ret)
	//{
		//ret = new double[ra * cb];
	//}
	for (int i = 0; i < ra; i++)
		for (int j = 0; j < cb; j++)
		{
		//double sum = a[i][0] * b[0][j];
		float sum = *(a + i * ca) * (*(b + j));
		for (int k = 1; k < ca; k++)
			//sum += a[i][k] * b[k][j];
			sum += *(a + i*ca + k) * (*(b + k*cb + j));
		//c[i][j] = sum;
		*(ret + i*cb + j) = sum;
		}

	//return ret;
}

//--------------------------------------------------------
//功能：求矩阵a的转置
//入口参数：矩阵首地址 a
//返回值：
//--------------------------------------------------------
//void Trans_Matrix(float *a, float *b, int ra, int ca)
//{
//	u8 i,j;

//}
