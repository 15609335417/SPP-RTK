#pragma once
#ifndef OutFile_H

#define OutFile_H
#define toStr(name)  (#name)

#include<stdio.h>
#include<iostream>
#include<fstream>
#include<iomanip>
#include"Spp.h"
#include"Decode.h"
#include"Coor.h"
#include "RTK.h"

//������
void _SetOutfileName();
void _OUTPostionFile(std::ofstream& ft, rover& rover);
void _OUTSatPostionFile(std::ofstream& ft, OBS& obs);
void _ErrorAnalysis(std::ofstream& ft, rover& rover, BASESTATION& base);

void _ResultFile(std::ofstream& ft, OBS& obs, rover& rover);
void _ConsoleResult(rover& rover);

//RTK�����Ļ���
void _OutRtkResultToConsole(RTKData& rtk);
//�ļ����
void _OutRtkResultToFile(std::ofstream& ft, RTKData& rtk);
//����������ͼ
void _OutRtkErrorToFile(std::ofstream& ft, RTKData& rtk);
#endif // !OutFile_H
