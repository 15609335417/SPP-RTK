#pragma once
#ifndef Matrix_H

#define Matrix_H
#define EPS 1e-10

#include<iostream>
#include<iomanip>
#include<stdio.h>
#include<stdlib.h>
#include"Vector.h"
//矩阵Matrix类
class Matrix
{
private:
	int rows = 0;
	int cols = 0;
	double** p;
	inline void initialize()
	{
		this->p = new double* [this->rows];//分配rows个指针
		for (int i = 0; i < this->rows; i++)
		{
			p[i] = new double[this->cols];//为p[i]进行动态内存分配，大小为cols
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
	}//单位阵
	inline ~Matrix()
	{
		for (int i = 0; i < rows; ++i)
		{
			delete[] p[i];
		}
		delete[] p;
	}
	//矩阵复制
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
	//矩阵赋值
	inline Matrix& operator=(const double* m)//用数组将数据传入矩阵(矩阵大小应声明）
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
			printf("矩阵维数不同，无法相乘！\n");
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
	
	//求矩阵的逆矩阵
	inline Matrix Inverse()
	{
		if (this->rows != this->cols)
		{
			std::cout << "只有方阵能求逆矩阵" << std::endl;
			std::abort();
			
		}
		double temp;
		Matrix A_B = Matrix(this->rows, this->cols);
		A_B = *this;//为矩阵A做一个备份
		Matrix B = Matrix(this->rows);
		//将小于EPS的数全部置0
		for (int i = 0; i < this->rows; i++) {
			for (int j = 0; j < this->cols; j++) {
				if (abs(this->p[i][j]) <= EPS) {
					this->p[i][j] = 0;
				}
			}
		}
		//选择需要互换的两行选主元
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
					printf("逆矩阵不存在\n");
					exit(-3);
					//abort();
					//break;
				}
			}
		}
		//通过初等行变换将A变为上三角矩阵
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
		//使对角元素均为1
		for (int i = 0; i < this->rows; i++) {
			temp = this->p[i][i];
			for (int j = 0; j < this->cols; j++) {
				this->p[i][j] = this->p[i][j] / temp;
				B.p[i][j] = B.p[i][j] / temp;
			}
		}
		//将已经变为上三角矩阵的A，变为单位矩阵
		for (int i = this->rows - 1; i >= 1; i--) {
			for (int j = i - 1; j >= 0; j--) {
				temp = this->p[j][i];
				for (int k = 0; k < this->cols; k++) {
					this->p[j][k] -= this->p[i][k] * temp;
					B.p[j][k] -= B.p[i][k] * temp;
				}
			}
		}
		*this = A_B;//还原A
		return B;//返回该矩阵的逆矩阵
	}
	//制造一个单位矩阵
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
	//实现矩阵的转置,
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
	//矩阵展示
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
	//矩阵求迹
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
	//矩阵行数
	inline int row()
	{
		return this->rows;
	}
	//矩阵列数
	inline int col()
	{
		return this->cols;
	}
	//修改某行某列的值
	inline void set(int rows, int cols, double value)
	{
		this->p[rows][cols] = value;
	}
};


#endif // !Matrix_H
