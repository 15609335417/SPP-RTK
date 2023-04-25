#include<iostream>
#include"Matrix.h"
using namespace std;


void matrix::initialize()
{
	this->p = new double* [this->rows];//����rows��ָ��
	for (int i = 0; i < this->rows; i++)
	{
		p[i] = new double[this->cols];//Ϊp[i]���ж�̬�ڴ���䣬��СΪcols
	}
}//��ʼ������

matrix::matrix(int rows, int cols)
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
matrix::matrix(int rows, int cols, double value)
{
	this->rows = rows;
	this->cols = cols;
	initialize();
	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			if(i==j)
				p[i][j] = value;
			else
			{
				p[i][j] = 0;
			}
		}
	}
}
matrix::matrix(const matrix& m)
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
matrix::matrix(int rows, int cols, const double* m)
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
matrix::matrix(int n, double value)
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
matrix::~matrix()
{
	for (int i = 0; i < rows; ++i)
	{
		delete[] p[i];
	}
	delete[] p;
}
//������
matrix& matrix::operator=(const matrix& m)
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
};//����ĸ���
//����ֵ
matrix& matrix::operator=(const double* m)//�����齫���ݴ������(�����СӦ������
{
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			p[i][j] = *(m + i * this->cols + j);
		}
	}
	return *this;
}//����ֵ
matrix matrix::operator+(const matrix& m)
{
	matrix n(this->rows, this->cols);
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			n.p[i][j] = this->p[i][j] + m.p[i][j];
		}
	}
	//*this = n;
	return n;
}
matrix matrix::operator-(const matrix& m)
{
	matrix n(this->rows, this->cols);
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			n.p[i][j] = this->p[i][j] - m.p[i][j];
		}
	}

	return n;
}
matrix matrix::operator*(const matrix& m) {
	matrix ba_M(this->rows, m.cols, 0.0);
	if (this->cols != m.rows)
		cout << "����ά����ͬ���޷���ˣ�" << endl;
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
matrix matrix::operator*(const double& m)
{
	matrix ba_M(this->rows,this->cols, 0.0);
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			ba_M.p[i][j] = m * this->p[i][j];
		}
	}
	return ba_M;
}

//�����������
matrix matrix::Inverse() {
	if (this->rows != this->cols)
	{
		std::cout << "ֻ�з������������" << std::endl;
		std::abort();
	}
	double temp;
	matrix A_B = matrix(this->rows, this->cols);
	A_B = *this;//Ϊ����A��һ������
	matrix B = eye(this->rows);
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
				cout << "����󲻴���\n";
				continue;
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
	/*cout << "�㷨�ɿ��Լ�⣬���ɿ��������λ����" << std::endl;
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			printf("%7.4lf\t\t", this->p[i][j]);
		}
		cout << endl;
	}
	cout << endl;
	*/
	*this = A_B;//��ԭA
	return B;//���ظþ���������

}
//����һ����λ����
matrix matrix::eye(int n) {
	matrix A(n, n);
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
//ʵ�־����ת��
matrix matrix::Transpose(){
	int col_size = this->cols;
	int row_size = this->rows;
	matrix mt(col_size, row_size);
	for (int i = 0; i < row_size; i++) {
		for (int j = 0; j < col_size; j++) {
			mt.p[j][i] = this->p[i][j];
		}
	}
	return mt;
}
//����չʾ
void matrix::Show() const
{
	for (int i = 0; i < this->rows; i++) {
		for (int j = 0; j < this->cols; j++) {
			cout << p[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}//������ʾ

double matrix::get(int rows, int cols)
{
	return this->p[rows][cols];
}

