#pragma once
#ifndef Matrix_H

#define Matrix_H
#define EPS 1e-10

#include<iostream>
#include<iomanip>
#include<stdio.h>
#include<stdlib.h>
#include"Vector.h"
//����Matrix��
class Matrix
{
private:
	int rows = 0;
	int cols = 0;
	double** p;
	inline void initialize()
	{
		this->p = new double* [this->rows];//����rows��ָ��
		for (int i = 0; i < this->rows; i++)
		{
			p[i] = new double[this->cols];//Ϊp[i]���ж�̬�ڴ���䣬��СΪcols
		}
	}
public:

	inline Matrix()
	{
		initialize();
	}
	inline Matrix(int rows, int cols)
	{
		this->rows = rows;
		this->cols = cols;
		initialize();
		for (int i = 0; i < this->rows; i++)
		{
			for (int j = 0; j < this->cols; j++)
			{
				p[i][j] = 0;
			}
		}
	}
	inline Matrix(int rows, int cols, double value)
	{
		this->rows = rows;
		this->cols = cols;
		initialize();
		for (int i = 0; i < this->rows; i++)
		{
			for (int j = 0; j < this->cols; j++)
			{
				p[i][j] = value;
			}
		}
	}
	inline Matrix(const Matrix& m)
	{
		this->rows = m.rows;
		this->cols = m.cols;
		initialize();
		for (int i = 0; i < m.rows; i++)
		{
			for (int j = 0; j < m.cols; j++)
			{
				p[i][j] = m.p[i][j];
			}
		}
	}
	inline Matrix(int rows, int cols, const double* m)
	{
		this->rows = rows;
		this->cols = cols;
		initialize();
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				p[i][j] = *(m + i * this->cols + j);
			}
		}
	}
	inline Matrix(int n, double value)
	{
		rows = n;
		cols = n;
		initialize();
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				if (i == j) {
					p[i][j] = value;
				}
				else {
					p[i][j] = 0;
				}
			}
		}
	}
	inline Matrix(int n)
	{
		rows = n;
		cols = n;
		initialize();
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				if (i == j) {
					p[i][j] = 1;
				}
				else {
					p[i][j] = 0;
				}
			}
		}
	}//��λ��
	inline ~Matrix()
	{
		for (int i = 0; i < rows; ++i)
		{
			delete[] p[i];
		}
		delete[] p;
	}
	//������
	inline Matrix& operator=(const Matrix& m)
	{
		if (this == &m)
			return *this;
		if (this->rows != m.rows || this->cols != m.cols)
		{
			for (int i = 0; i < rows; ++i) {
				delete[] p[i];
			}
			delete[] p;
			this->rows = m.rows;
			this->cols = m.cols;
			initialize();
		}
		for (int i = 0; i < this->rows; i++)
		{
			for (int j = 0; j < this->cols; j++)
			{
				this->p[i][j] = m.p[i][j];
			}
		}
		return *this;
	}
	//����ֵ
	inline Matrix& operator=(const double* m)//�����齫���ݴ������(�����СӦ������
	{
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				p[i][j] = *(m + i * this->cols + j);
			}
		}
		return *this;
	}
	inline Matrix operator+(const Matrix& m)
	{
		Matrix n(this->rows, this->cols);
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				n.p[i][j] = this->p[i][j] + m.p[i][j];
			}
		}
		return n;
	}
	inline Matrix operator-(const Matrix& m)
	{
		Matrix n(this->rows, this->cols);
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				n.p[i][j] = this->p[i][j] - m.p[i][j];
			}
		}
		return n;
	}
	inline Matrix operator*(const double& m)
	{
		Matrix ba_M(this->rows, this->cols, 0.0);
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				ba_M.p[i][j] = m * this->p[i][j];
			}
		}
		return ba_M;
	}
	inline Matrix operator*(const Matrix& m)
	{
		Matrix ba_M(this->rows, m.cols, 0.0);
		if (this->cols != m.rows)
			printf("����ά����ͬ���޷���ˣ�\n");
		else
		{
			for (int i = 0; i < this->rows; i++) {
				for (int j = 0; j < m.cols; j++) {
					for (int k = 0; k < this->cols; k++) {
						ba_M.p[i][j] += (this->p[i][k] * m.p[k][j]);
					}
				}
			}

		}
		return ba_M;
	}
	inline Vector operator*(const Vector& m)
	{
		Matrix M(this->rows, 1);
		Matrix N(this->cols, 1);
		for (int i = 0; i < this->cols; i++)
		{
			N.p[i][0] = m(i);
		}
		M = *this * N;
		Vector L(this->rows);
		double *arrayL =new double[this->rows];
		for (int i = 0; i < this->rows; i++)
		{
			arrayL[i] = M(i, 0);
		}
		L = arrayL;
		delete[] arrayL;
		return L;
	}
	inline double operator()(int rows, int cols)
	{
		return this->p[rows][cols];
	}
	
	//�����������
	inline Matrix Inverse()
	{
		if (this->rows != this->cols)
		{
			std::cout << "ֻ�з������������" << std::endl;
			std::abort();
			
		}
		double temp;
		Matrix A_B = Matrix(this->rows, this->cols);
		A_B = *this;//Ϊ����A��һ������
		Matrix B = Matrix(this->rows);
		//��С��EPS����ȫ����0
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				if (abs(this->p[i][j]) <= EPS) {
					this->p[i][j] = 0;
				}
			}
		}
		//ѡ����Ҫ����������ѡ��Ԫ
		for (int i = 0; i < this->rows; i++)
		{
			if (abs(this->p[i][i]) <= EPS)
			{
				bool flag = false;
				for (int j = 0; (j < this->rows) && (!flag); j++)
				{
					if ((abs(this->p[i][j]) > EPS) && (abs(this->p[j][i]) > EPS)) {
						flag = true;
						for (int k = 0; k < this->cols; k++) {
							temp = this->p[i][k];
							this->p[i][k] = this->p[j][k];
							this->p[j][k] = temp;
							temp = B.p[i][k];
							B.p[i][k] = B.p[j][k];
							B.p[j][k] = temp;
						}
					}
				}
				if (!flag) {
					printf("����󲻴���\n");
					exit(-3);
					//abort();
					//break;
				}
			}
		}
		//ͨ�������б任��A��Ϊ�����Ǿ���
		double temp_rate;
		for (int i = 0; i < this->rows; i++) {
			for (int j = i + 1; j < this->rows; j++) {
				temp_rate = this->p[j][i] / this->p[i][i];
				for (int k = 0; k < this->cols; k++) {
					this->p[j][k] -= this->p[i][k] * temp_rate;
					B.p[j][k] -= B.p[i][k] * temp_rate;
				}
				this->p[j][i] = 0;
			}
		}
		//ʹ�Խ�Ԫ�ؾ�Ϊ1
		for (int i = 0; i < this->rows; i++) {
			temp = this->p[i][i];
			for (int j = 0; j < this->cols; j++) {
				this->p[i][j] = this->p[i][j] / temp;
				B.p[i][j] = B.p[i][j] / temp;
			}
		}
		//���Ѿ���Ϊ�����Ǿ����A����Ϊ��λ����
		for (int i = this->rows - 1; i >= 1; i--) {
			for (int j = i - 1; j >= 0; j--) {
				temp = this->p[j][i];
				for (int k = 0; k < this->cols; k++) {
					this->p[j][k] -= this->p[i][k] * temp;
					B.p[j][k] -= B.p[i][k] * temp;
				}
			}
		}
		*this = A_B;//��ԭA
		return B;//���ظþ���������
	}
	//����һ����λ����
	inline Matrix eye(int n)
	{
		Matrix A(n, n);
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				if (i == j) {
					A.p[i][j] = 1;
				}
				else {
					A.p[i][j] = 0;
				}
			}
		}
		return A;
	}
	//ʵ�־����ת��,
	inline Matrix Transpose()
	{
		int col_size = this->cols;
		int row_size = this->rows;
		Matrix mt(col_size, row_size);
		for (int i = 0; i < row_size; i++) {
			for (int j = 0; j < col_size; j++) {
				mt.p[j][i] = this->p[i][j];
			}
		}
		return mt;
	}
	//����չʾ
	inline void Show() const
	{
		std::cout << std::fixed;
		std::cout << std::setprecision(4);
		std::cout.setf(std::ios::right);
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				std::cout << std::setw(13) << p[i][j] << " ";
				//std::cout  << p[i][j] << " ";
				//std::cout << std::setw(15) << std::setprecision(6) << p[i][j] << " ";
				//std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(6) << std::setiosflags(std::ios::right) << p[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout <<std::endl;
	}

	inline double get(int rows, int cols)
	{
		return this->p[rows][cols];

	}
	//������
	inline double tr()
	{
		double t = 0.0;
		if (this->rows == this->cols)
		{
			for (int i = 0; i < this->rows; i++)
			{
				t += this->p[i][i];
			}
		}
		return t;
	}
	//��������
	inline int row()
	{
		return this->rows;
	}
	//��������
	inline int col()
	{
		return this->cols;
	}
	//�޸�ĳ��ĳ�е�ֵ
	inline void set(int rows, int cols, double value)
	{
		this->p[rows][cols] = value;
	}
};


#endif // !Matrix_H
