#pragma once
#ifndef Vector_H

#define Vector_H
#define EPS 1e-10

#include <iostream>
#include<stdio.h>
#include<stdlib.h>
#include"Matrix.h"
class Vector
{
private:
	int rows=3;
	int cols=1;
	double* p;
	inline void initialize()
	{
		this->p = new double[this->rows];
	}

public:
	inline Vector()
	{
		initialize();
	}
	inline Vector(int rows)
	{
		this->rows = rows;
		this->cols = 1;
		initialize();
		for (int i = 0; i < this->rows; i++)
		{
			this->p[i] = 0;
		}
	}
	inline Vector(int rows,double value)
	{
		this->rows = rows;
		this->cols = 1;
		initialize();
		for (int i = 0; i < this->rows; i++)
		{
			this->p[i] = value;
		}
	}
	inline Vector(const Vector& m)
	{
		this->rows = m.rows;
		this->cols = 1;
		initialize();
		for (int i = 0; i < m.rows; i++)
		{
			this->p[i] = m.p[i];
		}
	}
	inline ~Vector()
	{
		delete[] p;
	}
	/// 重载= 赋值
	inline Vector& operator =(const double* a)
	{
		for (int i = 0; i < this->rows; i++)
		{
			p[i] = *(a + i);
		}
		return *this;
	}
	/// 重载= 复制
	inline Vector& operator=(const Vector& m)
	{
		if (this == &m)
			return *this;
		if (this->rows != m.rows || this->cols != m.cols)
		{
			delete[] p;
			this->rows = m.rows;
			this->cols = m.cols;
			initialize();
		}
		else if (this->rows == m.rows && this->cols == m.cols)
		{
			for (int i = 0; i < this->rows; i++)
			{
				p[i] = m.p[i];
			}
		}
		return *this;
	}
	inline Vector operator +(const Vector& a)
	{
		Vector res(this->rows);
		for (int i = 0; i < rows; i++)
		{
			res.p[i] = p[i] + a.p[i];
		}
		return res;
	}
	inline Vector operator +=(Vector& a)
	{
		Vector res(this->rows);
		res = *this + a;
		*this = res;
		return *this;
	}
	inline Vector operator-(const Vector& a)
	{
		Vector res(rows);
		if (this->rows == a.rows)
		{
			for (int i = 0; i < this->rows; i++)
			{
				res.p[i] = p[i] - a.p[i];
			}
		}
		return res;
	}
	inline Vector operator -=(Vector& a)
	{
		Vector res(this->rows);
		res = *this - a;
		*this = res;
		return *this;
	}
	//点积
	inline friend double DotProduct(Vector& a, Vector& b)
	{
		double res = 0;
		if (a.rows == b.rows)
		{
			for (int i = 0; i < a.rows; i++)
			{
				res += a.p[i] * b.p[i];
			}
		}
		else
		{
			printf("向量维数不同，不能做点乘运算！\n");
		}
		return res;
	}
	// 叉积
	inline Vector operator *(const Vector& a)
	{

		Vector res(3);
		if (a.rows == 3 && this->rows == 3)
		{
			res.p[0] = this->p[1] * a.p[2] - this->p[2] * a.p[1];
			res.p[1] = -(this->p[0] * a.p[2] - this->p[2] * a.p[0]);
			res.p[2] = this->p[0] * a.p[1] - this->p[1] * a.p[0];
		}
		else
		{
			printf("此程序只演示三维向量的叉乘！\n");
		}
		return res;
	}
	inline Vector operator *(const double a)
	{
		Vector m(this->rows);
		for (int i = 0; i < this->rows; i++)
			m.p[i] = this->p[i] * a;
		return m;
	}
	inline Vector operator /(const double a) 
	{
		Vector m(this->rows);
		for (int i = 0; i < rows; i++)
		{
			m.p[i] = p[i] / a;
		}
		return m;
	}
	inline Vector operator /(const Vector& a)
	{
		Vector m(this->rows);
		for (int i = 0; i < rows; i++)
		{
			m.p[i] = p[i] / a.p[i];
		}
		return m;
	}
	inline void Show() const
	{
		for (int i = 0; i < this->rows; i++) {
			printf("%20.10lf\n", this->p[i]);
		}
		//printf("\n");
	}
	inline double ModuleLength()const
	{
		return sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);

	}
	inline double get(int n)const
	{
		return p[n];
	}
	inline double operator()(int n) const
	{
		return this->p[n];
	}
	inline Vector operator()(int m,int n) const
	{
		Vector v(n - m + 1);
		for (int i = 0; i < (n - m+1); i++)
		{
			v.p[i] = this->p[m + i];
		}
		return v;
	}
	inline double Norm() const
	{
		double sum = 0.0;
		for (int i = 0; i < this->rows; i++)
		{
			sum += this->p[i] * this->p[i];
		}
		sum = sqrt(sum);
		return sum;
	}
	
};

#endif // !Vector_H
