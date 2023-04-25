#include"RTK.h"
#include"OutFile.h"
#include"Kalman.h"
using namespace std;

int main()
{	
	std::ofstream ft;
	std::ofstream fv; 
	//网络，基准站
	SOCKET baseSock;
	SOCKET roverSock;

	//串口，流动站
	CSerial cse;
	//基准站
	FILE* fs_base = NULL;
	unsigned char* buffer_base = new unsigned char[MaxNovNum];
	int n_base = 0;//每次要舍弃的字符数量

	//流动站
	FILE* fs_rover = NULL;
	unsigned char* buffer_rover = new unsigned char[MaxNovNum];
	int n_rover = 0;//每次要舍弃的字符数量
	
	//能否进行SPP/RTK
	short SPPStatus = 0;
	short RTKStatus = 0;
	short isFixed = -1;//0:OK
	//RTK观测值数据
	RTKData rtk;
	double fixedNo = 0;
	double fixedYes = 0;
	double fixedRate = 0.0;
	//kalman
	Kalman kalman;
	//SPP
	if (cfg.type == 0)
	{
		//0.输入文件
		if (cfg.mode == 0)
		{
			//解算文件为流动站数据
			if (fopen_s(&fs_rover, cfg.roverobsdatafile, "r+b"))
				printf("Rover obsation file open error!");
		}
		else if (cfg.mode == 1)//网络
		{
			if (!OpenSocket(baseSock, cfg.baseIPaddress, cfg.baseport))
				printf("Cannot open Socket！");
		}
		else//串口
		{
			if (!cse.Open(cfg.COMNo, cfg.baudrate))
				printf("Open Serial Error");
		}
    	//1.输出文件
	    ft.open(cfg.postionresultfile);
		//2.解算
		while ((cfg.mode != 0) || (cfg.mode == 0 && fs_rover != 0 && feof(fs_rover) != 1))
		{
			rtk.Roverrover = rover();
			if (cfg.mode == 0)
				SPPStatus =(short)_GetData(fs_rover, buffer_rover, n_rover, rtk.RoverObs,rtk);
			else if (cfg.mode == 1)
			{
				Sleep(999);
				SPPStatus = (short)_GetData(baseSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
			}
			else if (cfg.mode == 2)
			{
				Sleep(999);
				SPPStatus = (short)_GetData(cse, buffer_rover, n_rover, rtk.RoverObs, rtk);
			}
			if (SPPStatus == 1)
			{
				//流动站SPP
				_LinearCombination(rtk.RoverObs);
				/*SPP&SPV*/
				if (SPP(rtk.RoverObs, rtk.Eph, rtk.Roverrover))
					SPV(rtk.RoverObs, rtk.Eph, rtk.Roverrover);
				_ConsoleResult(rtk.Roverrover);
				//_ResultFile(ft,rtk.RoverObs,rtk.Roverrover);
				//_OUTPostionFile(ft, rtk.Roverrover);
			}
			else if (SPPStatus == 0)
				continue;
			else break;
		}
		if (cfg.mode == 1)
			CloseSocket(baseSock);
		else if (cfg.mode == 2)
			cse.Close();
		else;
	}
	else
	{
		
		if (cfg.mode == 0)   //文件
		{
			if (fopen_s(&fs_base, cfg.baseobsdatafile, "r+b"))
				printf("Base obsation file open error!");
			if (fopen_s(&fs_rover, cfg.roverobsdatafile, "r+b"))
				printf("Rover obsation file open error!");
			ft.open(cfg.postionresultfile);
			fv.open(cfg.postiondifffile);
		}
		else 
		{
			//打开网络
			if (!OpenSocket(baseSock, cfg.baseIPaddress, cfg.baseport))
				printf("Cannot open Socket！");
			if (!OpenSocket(roverSock, cfg.roverIPaddress, cfg.roverport))
				printf("Cannot open Socket！");
			ft.open(cfg.postionresultfile);
			fv.open(cfg.postiondifffile);
		}
		while ((cfg.mode != 0) || (cfg.mode == 0 && fs_base != 0 && feof(fs_base) != 1 && fs_rover != 0 && feof(fs_rover) != 1))
		{
			rtk.Baserover = rover();
			rtk.Roverrover = rover();
			if (cfg.mode == 0)
				RTKStatus = _TimeSynchronization(buffer_base, n_base, buffer_rover, n_rover, fs_base, fs_rover, rtk);
			else if (cfg.mode == 1)
			{
				Sleep(450);
				RTKStatus = _TimeSynchronization(buffer_base, n_base, buffer_rover, n_rover, baseSock, roverSock, rtk);
			}
			if (RTKStatus == 1)
			{
				//流动站SPP
				_LinearCombination(rtk.RoverObs);
				if (SPP(rtk.RoverObs, rtk.Eph, rtk.Roverrover))
					SPV(rtk.RoverObs, rtk.Eph, rtk.Roverrover);
				else
					continue;
				//基站spp
				_LinearCombination(rtk.BaseObs);
				if (SPP(rtk.BaseObs, rtk.Eph, rtk.Baserover))
					SPV(rtk.BaseObs, rtk.Eph, rtk.Baserover);
				else
					continue;

				rtk.gt = rtk.RoverObs.gt;
				//if (rtk.gt.SecOfWeek == 370593)
				//	printf("");
				//站间单差
				FormSDEpochObs(rtk.BaseObs, rtk.RoverObs, rtk.SDobs);
				//粗差探测
				DetectCycleSilp(rtk.SDobs);
				//参考星选取
				if (!DetRefSat(rtk.BaseObs, rtk.RoverObs, rtk.SDobs, rtk.DDObs)) 
					continue;

				//最小二乘
				if (cfg.method == 0)
				{
					if (!RTKFloat(rtk.BaseObs, rtk.RoverObs, rtk.basesta, rtk.Roverrover, rtk.SDobs, rtk.DDObs))
						continue;
					fixedNo++;

					////模糊度固定
					if (lambda((rtk.DDObs.DDSatNum[0] + rtk.DDObs.DDSatNum[1]) * 2, 2, rtk.DDObs.FloatAmb, rtk.DDObs.FloatAmbCov, rtk.DDObs.FixedAmb, rtk.DDObs.ResAmb))
						continue;
					rtk.DDObs.Ratio = rtk.DDObs.ResAmb[1] / rtk.DDObs.ResAmb[0];
					if (rtk.DDObs.Ratio - 3.0 > EPS)
					{
						rtk.DDObs.bFixed = true;
						fixedYes++;
					}
					else
						continue;
					////重新最小二乘
					//if (rtk.DDObs.bFixed)
					RTKFixed(rtk.BaseObs, rtk.RoverObs, rtk.basesta, rtk.Roverrover, rtk.SDobs, rtk.DDObs);
				}
				else if (cfg.method == 1)//kalman 滤波
				{
					if (!RTKFloatKalman(rtk.BaseObs, rtk.RoverObs, rtk.basesta, rtk.Roverrover, rtk.SDobs, rtk.DDObs, rtk.kalmanLast, rtk.kalmanNow))
						continue;
					fixedNo++;

					////模糊度固定
					if (lambda((rtk.DDObs.DDSatNum[0] + rtk.DDObs.DDSatNum[1]) * 2, 2, rtk.DDObs.FloatAmb, rtk.DDObs.FloatAmbCov, rtk.DDObs.FixedAmb, rtk.DDObs.ResAmb))
						continue;
					rtk.DDObs.Ratio = rtk.DDObs.ResAmb[1] / rtk.DDObs.ResAmb[0];

					if (rtk.DDObs.Ratio - 3.0 > EPS)
					{
						rtk.DDObs.bFixed = true;
						fixedYes++;
					}
					////else
					////	continue;

					if (!RTKFixedKalman(rtk.BaseObs, rtk.RoverObs, rtk.basesta, rtk.Roverrover, rtk.SDobs, rtk.DDObs, rtk.kalmanLast))
						continue;

				}
				_OutRtkResultToFile(ft, rtk);
				_OutRtkErrorToFile(fv, rtk);
				_OutRtkResultToConsole(rtk);	
				//fixedRate = fixedYes / fixedNo;
				//printf("%lf%%\n", fixedRate*100);

			}
			else if (RTKStatus == 2)
			{
				//流动站SPP
				_LinearCombination(rtk.RoverObs);
				/*SPP&SPV*/
				if (SPP(rtk.RoverObs, rtk.Eph, rtk.Roverrover))
					SPV(rtk.RoverObs, rtk.Eph, rtk.Roverrover);

			}
			else if (RTKStatus == 0)
				continue;
			else break;

		}
		if (cfg.mode != 0)
		{
			CloseSocket(baseSock);
			CloseSocket(roverSock);
		}
	}
	fixedRate = fixedYes / fixedNo;
	printf("%lf\n", fixedRate);
	ft.close();
	fv.close();
	delete[] buffer_base;
	delete[] buffer_rover;
	return 0;

}

