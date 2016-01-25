#include <stdio.h>
#include "matrix.h"
//--------------------------------------------------------
//���ܣ������ n X n ������ʽ
//��ڲ����������׵�ַ p���������� n
//����ֵ�����������ʽֵ
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

//������Եĸ���
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
//���ܣ���k��k������Ԫ��A(mn)�Ĵ�������ʽ
//��ڲ�����k��k�����׵�ַ��Ԫ��A���±�m,n; �������� k
//����ֵ�� k��k������Ԫ��A(mn)�Ĵ�������ʽ
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
//���ܣ���k��k����İ������
//��ڲ�����m��k��k�����׵�ַ���������� k���������adj�ǰ���������ڵ�ַ
//����ֵ�� ��
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
//���ܣ���k��k����������
//��ڲ�����m��k��k�����׵�ַ���������� k���������inv����������ڵ�ַ
//����ֵ�� ��
//----------------------------------------------------
void inverse_matrix(float *raw, float *inv, int k)
{
	float det = determinant(raw, k); //������ʽ
	//if (det == 0)
	//{
	//	cout << "���󲻿���" << endl;
	//	return;
	//}
	adjoint_m(raw, inv, k); //��������
	int len = k * k;
	for (int i = 0; i < len; i++)
		*(inv + i) /= det;
}


//--------------------------------------------------------
//���ܣ������a��b����˽��
//��ڲ����������׵�ַ a��b������a����ra������rc������b������rb������cb
//����ֵ������a��b����˽��
//--------------------------------------------------------
void m_multiply(float *a, float *b, float *c, int ra, int ca, int rb, int cb)
{
	//if (ca != rb)
	//{
	//	cout << "���󲻿ɳ�" << endl;
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
//���ܣ������a��ת��
//��ڲ����������׵�ַ a
//����ֵ��
//--------------------------------------------------------
//void Trans_Matrix(float *a, float *b, int ra, int ca)
//{
//	u8 i,j;

//}
