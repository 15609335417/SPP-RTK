#pragma once
#ifndef Config_H

#define Config_H

#include<iostream>
#include<stdio.h>

struct Config
{
	short type;                   //�������ͣ�0��SPP��1��RTK
	short mode;                   //����ģʽ��0���ļ���1�����磻2������
	short method;                 //���㷽����0��LSQ��1��Kalman 
	short COMNo;                  //���ں� 
	int baudrate;				  //������
	char baseIPaddress[24];       //����IP��ַ
	short baseport;               //�˿ں�
	char roverIPaddress[24];      //����IP��ַ
	short roverport;              //�˿ں�
	char baseobsdatafile[256];    //�۲��ļ�·��
	char roverobsdatafile[256];   //����վ�۲�ֵ�ļ�
	char postionresultfile[256];  //λ�ý���ļ�·��
	char postiondifffile[256];    //λ�ý������ļ�·��
	double pseudorangenoise;      //α��۲�ֵ����
	double eleuationmask;         //��ֹ�߶Ƚ�
	Config()
	{
		mode             = 0;
		COMNo            = 0;
		baudrate         = 0;
		baseport         = 0;
		roverport        = 0;
		pseudorangenoise = 0.0;
		eleuationmask    = 0.0;
	}
	Config(const char* filename)
	{
		FILE* fp = NULL;
		char buffer[256];
		if (fopen_s(&fp, filename, "r+b"))
		{
			printf("Open Error! Please check out .cfg!");
			return;
		}
		//���ж�ȡ�����ļ�
		fgets(buffer, 256, fp);
		fgets(buffer, 256, fp);         type                      = atoi(buffer + 32);
		fgets(buffer, 256, fp);         mode                      = atoi(buffer + 32);
		fgets(buffer, 256, fp);         method                    = atoi(buffer + 32);
		fgets(buffer, 256, fp);         COMNo                     = atoi(buffer + 32);
										baudrate                  = atoi(buffer + 34);
		fgets(buffer, 256, fp);         memcpy(baseIPaddress, buffer + 32, 14);
										baseport = atoi(buffer + 47);
		fgets(buffer, 256, fp);         memcpy(roverIPaddress, buffer + 32, 14);
		                                roverport = atoi(buffer + 47);
		fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", baseobsdatafile, 256);
		fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", roverobsdatafile, 256);
		fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", postionresultfile, 256);
		fgets(buffer, 256, fp);         sscanf_s(buffer + 32, "%s", postiondifffile, 256);
		fgets(buffer, 256, fp);         pseudorangenoise          = atof(buffer + 32);
		fgets(buffer, 256, fp);         eleuationmask             = atof(buffer + 32);

		fclose(fp);

	}
};

extern int liyuan;
extern Config cfg;
Config _SetConfig(const char* filename);


#endif // !Config_H
