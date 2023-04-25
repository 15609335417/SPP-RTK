#pragma once
#ifndef Config_H

#define Config_H

#include<iostream>
#include<stdio.h>

struct Config
{
	short type;                   //解算类型：0：SPP；1：RTK
	short mode;                   //解算模式：0：文件；1：网络；2：串口
	short method;                 //解算方法：0：LSQ；1：Kalman 
	short COMNo;                  //串口号 
	int baudrate;				  //波特率
	char baseIPaddress[24];       //网络IP地址
	short baseport;               //端口号
	char roverIPaddress[24];      //网络IP地址
	short roverport;              //端口号
	char baseobsdatafile[256];    //观测文件路径
	char roverobsdatafile[256];   //流动站观测值文件
	char postionresultfile[256];  //位置结果文件路径
	char postiondifffile[256];    //位置结果误差文件路径
	double pseudorangenoise;      //伪距观测值噪声
	double eleuationmask;         //截止高度角
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
		//逐行读取配置文件
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
