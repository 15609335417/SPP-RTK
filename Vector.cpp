#include<iostream>
#include"Matrix.h"

using namespace std;
void vector::initialize()
{
	this->p = new double[this->rows];
}
vector::vector(int rows)
 {
	 this->rows = rows;
	 this->cols = 1;
	 initialize();
	 for (int i = 0; i < this->rows; i++)
	 {
		 this->p[i] = 0;
	 }
 }
vector::vector(const vector& m)
{
	this->rows = m.rows;
	this->cols = 1;
	initialize();
	for (int i = 0; i < m.rows; i++)
	{
		this->p[i] = m.p[i];
	}
}
vector::~vector()
{
	delete[] p;
}
//赋值
vector& vector::operator =(const double* a)
{
	
	for (int i = 0; i < this->rows; i++)
	{
		p[i] = *(a + i);
	}
	return *this;
}
/// <summary>
/// 重载= 复制
/// </summary>
vector& vector::operator=(const vector& m)
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
vector vector::operator +(const vector& a)
{
	vector res(this->rows);
	for (int i = 0; i < rows; i++)
	{
		res.p[i] = p[i] + a.p[i];
	}
	return res;
}
vector vector::operator-(const vector& a)
{
	vector res(rows);
	if (this->rows == a.rows)
	{
		for (int i = 0; i < this->rows; i++)
		{
			res.p[i] = p[i] - a.p[i];
		}
	}
	return res;
}
//点积
double DotProduct( vector& a,  vector& b)
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
		cout << "向量维数不同，不能做点乘运算！" << endl;
	}
	return res;
}
vector vector::operator *(const vector& a)
{
	//此处只演示三维向量叉乘

	vector res(3);
	if (a.rows == 3 && this->rows == 3)
	{
		res.p[0] = this->p[1] * a.p[2] - this->p[2] * a.p[1];
		res.p[1] = -(this->p[0] * a.p[2] - this->p[2] * a.p[0]);
		res.p[2] = this->p[0] * a.p[1] - this->p[1] * a.p[0];

	}
	else
	{
		cout << "此程序只演示三维向量的叉乘！" << endl;
	}
	//*this = res;
	return res;
}
vector vector::operator /(const double a)
{
	for (int i = 0; i < rows; i++)
	{
		p[i] = p[i] / a;
	}
	return *this;
}

void vector::Show() const
{
	for (int i = 0; i < this->rows; i++) {
		cout << this->p[i] << " ";
	}
	cout << endl;
}

double vector::ModuleLength()const
{
	return sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}
double vector::get(int n)const
{
	return p[n];
}

