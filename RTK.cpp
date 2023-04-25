#include"RTK.h"
/// <summary>
/// 时间对齐
/// 流动站时间-基准站时间<2s?
/// 文件读取完毕就返回true
/// </summary>
/// <param name="buffer"></param>
/// <param name=""></param>
/// <param name="fp"></param>
/// <param name="rtk"></param>
/// <returns></returns>
short _TimeSynchronization(
	unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, 
	int& n_rover, FILE* fp_base, FILE* fp_rover, RTKData& rtk
)
{
	short RoverStatus = 0;
	short BaseStatus = 0;
	double interval = 0.1;
	//先读取流动站文件，解码
	//while获取rover数据
	while (RoverStatus == 0)
		RoverStatus = _GetData(fp_rover, buffer_rover, n_rover, rtk.RoverObs, rtk);
	double delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;

	while (fabs(delta_time) - interval > EPS)
	{
		if (delta_time + interval < EPS)//基站在前，流动站在后面，读取流动站，并给出单点定位结果
		{
			if (RoverStatus == 1)
				RoverStatus = _GetData(fp_rover, buffer_rover, n_rover, rtk.RoverObs, rtk);
			while (RoverStatus == 0)
				RoverStatus = _GetData(fp_rover, buffer_rover, n_rover, rtk.RoverObs, rtk);
			delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
		}
		else
		{
			if (BaseStatus == 1)
				BaseStatus = _GetData(fp_base, buffer_base, n_base, rtk.BaseObs, rtk);
			while (BaseStatus == 0)
				BaseStatus = _GetData(fp_base, buffer_base, n_base, rtk.BaseObs, rtk);
			delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
		}
	}

	//时间同步成功条件
	// 返回0: 不同步
	// 返回1：同步
	// 返回2：只有流动站
	// 返回-1：文件读取结束
	//时间同步成功返回true，不成功就返回false
	if (RoverStatus == -1)
		return -1;
	else if (BaseStatus == 1 && RoverStatus == 1)
		return 1;
	else if (BaseStatus != 1 && RoverStatus == 1)
		return 2;
	else if (fabs(delta_time) - 0.1 < EPS)
		return 0;
	else;
}

/// <summary>
/// 网络与串口数据对齐
/// </summary>
/// <param name="buffer_base"></param>
/// <param name="n_base"></param>
/// <param name="buffer_rover"></param>
/// <param name="n_rover"></param>
/// <param name="cse">串口</param>
/// <param name="sock">网络</param>
/// <param name="rtk"></param>
/// <returns></returns>
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover, CSerial& cse, SOCKET& sock, RTKData& rtk)
{
	short RoverStatus = 0;//通过串口获取
	short BaseStatus = 0;//通过网络获取
	double interval = 0.1;//两者相差多少时间合理
	//先读串口
	//先读取流动站文件，解码
	//while获取rover数据
	while (RoverStatus == 0)
		RoverStatus = _GetData(cse, buffer_rover, n_rover, rtk.RoverObs, rtk);
	double delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
	while (fabs(delta_time) - interval > EPS)
	{
		if (delta_time + interval < EPS)//基站在前，流动站在后面，读取流动站，并给出单点定位结果
		{
			if (RoverStatus == 1)
				RoverStatus = _GetData(cse, buffer_rover, n_rover, rtk.RoverObs, rtk);
			while (RoverStatus == 0)
				RoverStatus = _GetData(cse, buffer_rover, n_rover, rtk.RoverObs, rtk);
			delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
		}
		else
		{
			if (BaseStatus == 1)
				BaseStatus = _GetData(sock, buffer_base, n_base, rtk.BaseObs, rtk);
			while (BaseStatus == 0)
				BaseStatus = _GetData(sock, buffer_base, n_base, rtk.BaseObs, rtk);
			delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
		}
	}
	//时间同步成功条件
	// 返回0: 不同步
	// 返回1：同步
	// 返回2：只有流动站
	// 返回-1：文件读取结束
	//时间同步成功返回true，不成功就返回false
	if (RoverStatus == -1)
		return -1;
	else if (BaseStatus == 1 && RoverStatus == 1)
		return 1;
	else if (BaseStatus != 1 && RoverStatus == 1)
		return 2;
	else if (fabs(delta_time) - 0.1 < EPS)
		return 0;
	else;

}

/// <summary>
/// 实时时间同步
/// </summary>
/// <param name="buffer_base"></param>
/// <param name="n_base"></param>
/// <param name="buffer_rover"></param>
/// <param name="n_rover"></param>
/// <param name="baseSock"></param>
/// <param name="roverSock"></param>
/// <param name="rtk"></param>
/// <returns></returns>
short _TimeSynchronization(
	unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover,
	int& n_rover, SOCKET& baseSock, SOCKET& roverSock, RTKData& rtk
) {
	short RoverStatus = 0;//通过串口获取
	short BaseStatus = 0;//通过网络获取
	double interval = 0;//两者相差多少时间合理
	//先读串口
	//先读取流动站文件，解码
	//while获取rover数据
	RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
	BaseStatus = _GetData(baseSock, buffer_base, n_base, rtk.BaseObs, rtk);
	//while (RoverStatus == 0)
	//	RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
	double delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
	while (fabs(delta_time) > interval )
	{
		if (delta_time < -interval)//基站在前，流动站在后面，读取流动站，并给出单点定位结果
		{
			RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);

			//if (RoverStatus == 1)
			//	RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
			//while (RoverStatus == 0)
			//	RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
			delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
		}
		else
		{
			BaseStatus = _GetData(baseSock, buffer_base, n_base, rtk.BaseObs, rtk);

			//if (BaseStatus == 1)
			//	BaseStatus = _GetData(baseSock, buffer_base, n_base, rtk.BaseObs, rtk);
			//while (BaseStatus == 0)
			//	BaseStatus = _GetData(baseSock, buffer_base, n_base, rtk.BaseObs, rtk);
			delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
		}
	}
	//时间同步成功条件
	// 返回0: 不同步
	// 返回1：同步
	// 返回2：只有流动站
	// 返回-1：文件读取结束
	//时间同步成功返回true，不成功就返回false
	if (RoverStatus == -1)
		return -1;
	else if (BaseStatus == 1 && RoverStatus == 1)
		return 1;
	else if (BaseStatus != 1 && RoverStatus == 1)
		return 2;
	else if (fabs(delta_time) - 0.1 < EPS)
		return 0;
	else;
}


/// <summary>
/// 数据获取
/// </summary>
/// <param name="buffer"></param>
/// <param name="n"></param>
/// <param name="obs"></param>
/// <param name="rtk"></param>
/// <param name="rover"></param>
/// <returns>1:获取数据成功</returns>
/// <returns>0：获取数据不成功</returns>
/// <returns>-1：文件读取结束</returns>
short _GetData(FILE* fp, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk)
{
	//读取数据fread
	int LenRead = (int)fread(buffer + n, sizeof(char), MaxNovNum - n, fp);
	if (LenRead < (MaxNovNum - n))
	{
		printf("Finished!\n"); fclose(fp); //break;
		return -1;
	}
	return (short)_DecodeNovOem7(buffer, n, obs, rtk.Eph, rtk.basesta);
}

short _GetData(CSerial& cse, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk)
{
	int LenRead_rover = cse.ReadData(buffer, MaxNovNum);
	n = n + LenRead_rover;
	return (short)_DecodeNovOem7(buffer, n, obs, rtk.Eph, rtk.basesta);
}

short _GetData(SOCKET& sock, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk)
{
	int LenRead_base = recv(sock, (char*)buffer + n, MaxNovNum - n, 0);
	n = n + LenRead_base;
	return (short)_DecodeNovOem7(buffer, n, obs, rtk.Eph, rtk.basesta);
}

/// <summary>
/// 站间单差：流动站减基准站
/// </summary>
/// <param name="baseObs">基站观测值</param>
/// <param name="roverObs">流动站观测值</param>
/// <param name="SDObs">单差观测值</param>
void FormSDEpochObs(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs)
{
	SDSATOBS SDSatObs[MaxSatNum];
	memcpy(SDObs.SdSatObs, SDSatObs, sizeof(SDSATOBS) * MaxSatNum);
	//伪距
	int num = 0;
	for (int i = 0; i < roverObs.SatNum; i++)
	{
		for (int j = 0; j < baseObs.SatNum; j++)
		{
			if (roverObs.satobs[i].system == baseObs.satobs[j].system && roverObs.satobs[i].PRN == baseObs.satobs[j].PRN)
			{
				if (fabs(roverObs.satobs[i].L[0]) < EPS || fabs(baseObs.satobs[j].L[0]) < EPS || fabs(roverObs.satobs[i].L[1]) < EPS || fabs(baseObs.satobs[j].L[1]) < EPS)
					break;//本颗卫星无双频观测值数据
				if (roverObs.satpos[i].Status == 1 && baseObs.satpos[j].Status == 1)//卫星位置与卫星观测值索引一致
					SDObs.SdSatObs[num].Valid = 0;//站间单差数据有效
				else continue;
				for (int k = 0; k < 2; k++) {
					SDObs.SdSatObs[num].dP[k] = roverObs.satobs[i].C[k] - baseObs.satobs[j].C[k];
					SDObs.SdSatObs[num].dL[k] = roverObs.satobs[i].L[k] - baseObs.satobs[j].L[k];
					SDObs.SdSatObs[num].f[k] = roverObs.satobs[i].f[k];
					
				}
				SDObs.SdSatObs[num].nBas = j;
				SDObs.SdSatObs[num].nRov = i;
				SDObs.SdSatObs[num].PRN = roverObs.satobs[i].PRN;
				SDObs.SdSatObs[num].system = roverObs.satobs[i].system;
				
				num++;
				break;
			}
			else continue;
		}
	}
	SDObs.SatNum = num;
	SDObs.gt = roverObs.gt;
}

/// <summary>
/// 粗差探测
/// </summary>
/// <param name="SDObs"></param>
void DetectCycleSilp(SDEPOCHOBS& SDObs)
{
	MWGF outer[MaxSatNum];//本历元的MW GF 组合
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid == -1) continue;
		outer[i].system = SDObs.SdSatObs[i].system;
		outer[i].PRN = SDObs.SdSatObs[i].PRN;

		//计算组合Lmw，Lgf
		outer[i].Lmw = 1 / (SDObs.SdSatObs[i].f[0] - SDObs.SdSatObs[i].f[1]) * (SDObs.SdSatObs[i].f[0] * SDObs.SdSatObs[i].dL[0] - SDObs.SdSatObs[i].f[1] * SDObs.SdSatObs[i].dL[1])
			- 1.0 / (SDObs.SdSatObs[i].f[0] + SDObs.SdSatObs[i].f[1]) * (SDObs.SdSatObs[i].f[0] * SDObs.SdSatObs[i].dP[0] + SDObs.SdSatObs[i].f[1] * SDObs.SdSatObs[i].dP[1]);
		outer[i].Lgf = SDObs.SdSatObs[i].dL[0] - SDObs.SdSatObs[i].dL[1];

		//突然出现的问题的解决
		bool Status = false;
		//找上一时刻的PRN
		for (int j = 0; j < MaxSatNum; j++)
		{
			if (SDObs.mwgf[j].PRN == 0)continue;
			if (outer[i].system != SDObs.mwgf[j].system)continue;
			if (outer[i].PRN != SDObs.mwgf[j].PRN)continue;
			//找到j了 j 上一时刻
			Status = true;
			//前后历元的_Lmw Lgf作差
			double dGF = outer[i].Lgf - SDObs.mwgf[j].Lgf;
			double dMW = outer[i].Lmw - SDObs.mwgf[j]._Lmw;
			//检查是否超限
			if (fabs(SDObs.mwgf[j]._Lmw) > EPS && fabs(dMW) <= MaxDeltaLmw && fabs(dGF) <= MaxDeltaLgf)
			{
				//标记为可用
				SDObs.SdSatObs[i].Valid = 1;//?可以这么写吗
			}
			else
			{
				//粗差出现即标记为2，令本时刻的平滑值为计算所得Lmw
				outer[i]._Lmw = outer[i].Lmw;
				outer[i].n = 1;
				SDObs.SdSatObs[i].Valid = 2;//双频观测值且有周跳
				break;
			}
			//计算本历元的_Lmw[1];
			outer[i]._Lmw = (double)(SDObs.mwgf[j].n - 1) / (double)SDObs.mwgf[j].n * SDObs.mwgf[j]._Lmw + 1.0 / (double)SDObs.mwgf[j].n * outer[i].Lmw;
			outer[i].n = SDObs.mwgf[j].n + 1;
			break;
		}
		if (!Status)
		{
			SDObs.SdSatObs[i].Valid = 3;//突然出现的卫星
			outer[i]._Lmw = outer[i].Lmw;
			outer[i].n = 1;//本时刻为第一个历元
		}
	}
	memcpy(SDObs.mwgf, outer, MaxSatNum * sizeof(MWGF));
}

/// <summary>
/// 参考星选取
/// </summary>
/// <param name="baseObs"></param>
/// <param name="roverObs"></param>
/// <param name="SDObs"></param>
/// <param name="DDCObs"></param>
bool DetRefSat(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs)
{
	//参考星选取
	//上一时刻的参考星
	int RefPrn[2] = { DDCObs.RefPrn[0] ,DDCObs.RefPrn[1] };
	DDCObs = DDCOBS();
	double CN0L1[2], CN0L2[2];
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		else DDCObs.Sats++; //0.通过粗差探测且有双频观测值数据以及有卫星位置
		if (SDObs.SdSatObs[i].system == GPS) {
			

			if (baseObs.satobs[SDObs.SdSatObs[i].nBas].ParityKnownFlag[0] == 0 || baseObs.satobs[SDObs.SdSatObs[i].nBas].ParityKnownFlag[1] == 0) continue;
			else if (roverObs.satobs[SDObs.SdSatObs[i].nRov].ParityKnownFlag[0] == 0 || roverObs.satobs[SDObs.SdSatObs[i].nRov].ParityKnownFlag[1] == 0)continue;
			else
				SDObs.SdSatObs[i].half = 1;//2.根据有没有半周

			DDCObs.DDSatNum[0]++;//GPS 参与双差观测值的卫星数

			//1.上一历元作为参考星可直接成为本历元参考星
			if (RefPrn[0] == SDObs.SdSatObs[i].PRN&&RefPrn[0]>0) {
				DDCObs.RefPrn[0] = SDObs.SdSatObs[i].PRN;
				DDCObs.RefPos[0] = i;
				DDCObs.RefStatus[0] = 1; 
				continue;
			}

			if (DDCObs.RefStatus[0] == 1)continue;//上述方法成功的话就选该卫星为参考星


			if (baseObs.satobs[SDObs.SdSatObs[i].nBas].CNR[0] - 40.0 < EPS || baseObs.satobs[SDObs.SdSatObs[i].nBas].CNR[1] - 28.0 < EPS) continue;
			else if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] - 40.0 < EPS || roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] - 28.0 < EPS) continue;
			else
			{
				//3.根据CN0判断
				if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] > CN0L1[0] && roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] > CN0L2[0])
				{
					DDCObs.RefPrn[0] = SDObs.SdSatObs[i].PRN;
					DDCObs.RefPos[0] = i;
					CN0L1[0] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0];
					CN0L2[0] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1];
				}

			}

		}
		else if (SDObs.SdSatObs[i].system == BDS)
		{
			
			if (baseObs.satobs[SDObs.SdSatObs[i].nBas].ParityKnownFlag[0] == 0 || baseObs.satobs[SDObs.SdSatObs[i].nBas].ParityKnownFlag[1] == 0) continue;
			else if (roverObs.satobs[SDObs.SdSatObs[i].nRov].ParityKnownFlag[0] == 0 || roverObs.satobs[SDObs.SdSatObs[i].nRov].ParityKnownFlag[1] == 0)continue;
			else
				SDObs.SdSatObs[i].half = 1;//2.根据有没有半周

			DDCObs.DDSatNum[1]++;//BDS 参与双差观测值的卫星数

			//1.上一历元作为参考星可直接成为本历元参考星
			if (RefPrn[1] == SDObs.SdSatObs[i].PRN && RefPrn[1] > 0) {
				DDCObs.RefPrn[1] = SDObs.SdSatObs[i].PRN;
				DDCObs.RefPos[1] = i;
				DDCObs.RefStatus[1] = 1;
				continue;
			}
			 
			if (DDCObs.RefStatus[1] == 1)continue;


			if (baseObs.satobs[SDObs.SdSatObs[i].nBas].CNR[0] - 40.0 < EPS || baseObs.satobs[SDObs.SdSatObs[i].nBas].CNR[1] - 28.0 < EPS) continue;
			else if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] - 40.0 < EPS || roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] - 28.0 < EPS) continue;
			else
			{
				//3.CN0
				if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] > CN0L1[1] && roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] > CN0L2[1])
				{
					DDCObs.RefPrn[1] = SDObs.SdSatObs[i].PRN;
					DDCObs.RefPos[1] = i;
					CN0L1[1] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0];
					CN0L2[1] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1];
				}
			}
		}
	}
	//没有找到高载噪比的基准星
	//到时候有bug只能在测试
	CN0L1[0] = CN0L1[1] = CN0L2[0] = CN0L2[1] = 0.0;
	if (DDCObs.RefPos[0] < 0)
	{
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;//0.通过粗差探测且有双频观测值数据以及有卫星位置
			if (SDObs.SdSatObs[i].half == 0)continue;//2.有半周的不选
			if (SDObs.SdSatObs[i].system == GPS)
			{
				if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] > CN0L1[0] && roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] > CN0L2[0])
				{
					DDCObs.RefPrn[0] = SDObs.SdSatObs[i].PRN;
					DDCObs.RefPos[0] = i;
					CN0L1[0] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0];
					CN0L2[0] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1];
				}

			}
			else break;
		}

	}
	else if (DDCObs.RefPos[1] < 0)
	{
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;//0.通过粗差探测且有双频观测值数据以及有卫星位置
			if (SDObs.SdSatObs[i].half == 0)continue;//2.有半周的不选
			if (SDObs.SdSatObs[i].system == BDS)
			{
				if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] > CN0L1[0] && roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] > CN0L2[0])
				{
					DDCObs.RefPrn[1] = SDObs.SdSatObs[i].PRN;
					DDCObs.RefPos[1] = i;
					CN0L1[1] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0];
					CN0L2[1] = roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1];
				}

			}
			else continue;
		}
	}
	//找到之后做个标记，标记每次都会刷新
	if (DDCObs.RefPos[0] >= 0) { DDCObs.RefStatus[0] = 1; DDCObs.DDSatNum[0]--; }
	if (DDCObs.RefPos[1] >= 0) { DDCObs.RefStatus[1] = 1; DDCObs.DDSatNum[1]--; }
	if (DDCObs.RefPos[0] == -1 && DDCObs.RefPos[1] == -1) return false;
	else return true;

}

/// <summary>
/// 相对定位浮点解
/// </summary>
/// <param name="eph"></param>
/// <param name="base"></param>
/// <param name="rover"></param>
/// <param name="SDObs"></param>
/// <param name="DDCObs"></param>
/// <returns></returns>
bool RTKFloat(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs)
{
	/*if (SDObs.gt.SecOfWeek == 370126)
		printf(" ");*/
	//1.设置基站和流动站初值
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion = rover._XYZpostion;
	//2.计算GPS和BDS的双差卫星数
	//3/4计算站坐标到所有卫星的几何距离
	//按单差观测值数组的索引存，没有通过粗差探测的没有距离
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];

	//5.对单差观测值进行循环
	const int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int row = 0;//带入循环的变量
	const int cols = 3 + 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int col = 0;//用来处理波长
	if (rows < cols) return false;//行数小于列数不予计算
	//模糊度初值
	double* N = new double[cols - 3];//模糊度初值 //需进行赋值 //相位减伪距
	int Ncol = 0;
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;//有半周的不参与RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1])continue;
		//!!!!!!双差相位减双差伪距  除以波长
		for (int k = 0; k < 2; k++)
		{
			if (SDObs.SdSatObs[i].system == GPS)
			{
				N[Ncol] = ((SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k])
					- (SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dP[k]))
					/ (Cspeed / SDObs.SdSatObs[i].f[k]);
				Ncol++;
			}
			else if (SDObs.SdSatObs[i].system == BDS)
			{
				N[Ncol] = ((SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k])
					- (SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dP[k]))
					/ (Cspeed / SDObs.SdSatObs[i].f[k]);
				Ncol++;
			}
			else continue;
			
		}
	}
	double normX = 1.0;
	int count = 0;
	double* arrayB = new double[rows * cols];//防止数组越界，多加一行//4*(GPS 双差 卫星数+BDS 双差 卫星数) 行 (3+2*(GPS 双差 卫星数+BDS 双差 卫星数))列
	for (int i = 0; i < rows * cols; i++)
		arrayB[i] = 0.0;
	double* arrayW = new double[rows];//w矩阵，亦或者w向量
	for (int i = 0; i < rows; i++)
		arrayW[i] = 0.0;
	double* arrayP = new double[rows * rows];//防止数组越界
	double sigmaP = 0.5;
	double sigmaL = 0.0005;//单位都是m
	//sigmaP = pow(sigmaP, 2);
	//sigmaL = pow(sigmaL, 2);
	for (int i = 0; i < rows * rows ; i++)
		arrayP[i] = 0.0;
	//协方差阵
	Matrix Q(cols, cols);
	while (normX > 1E-7)
	{
		row = 0;
		col = 0;
		//计算站与星之间的距离
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half != 1)continue;//有半周的不参与RTk
			PR[i] = distance(roverPostion, roverObs.satpos, SDObs.SdSatObs[i].nRov);
			PB[i] = distance(basePostion, baseObs.satpos, SDObs.SdSatObs[i].nBas);
		}
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half !=1)continue;//有半周的不参与RTk
			if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//基准星不做差
			if (SDObs.SdSatObs[i].system == GPS)
			{
				if (!DDCObs.RefStatus[0]) continue;//没有选到基准星的大概率是单差观测值没有通过粗差检验
				//B矩阵
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.x) / PR[DDCObs.RefPos[0]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.y) / PR[DDCObs.RefPos[0]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.z) / PR[DDCObs.RefPos[0]];
				///每四行一次循环
				//第一、二行
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dP[k]
						- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]]);
					arrayP[rows * row + row] = DDCObs.DDSatNum[0] / (2 * sigmaP * (DDCObs.DDSatNum[0] + 1));//主对角线，即方差
					for (int j = 0; j < DDCObs.DDSatNum[0] * 4; j = j + 1)//协方差
					{
						if (rows * row + j == rows * row + row)continue;
						if (abs(rows * row + j - (rows * row + row)) % 4 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaP * (DDCObs.DDSatNum[0] + 1));
					}
					row++;
				}
				//第三、四行
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayB[cols * row + 3 + col] = Cspeed / SDObs.SdSatObs[i].f[k];
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[0] / (2 * sigmaL * (DDCObs.DDSatNum[0] + 1));
					for (int j = 0; j < DDCObs.DDSatNum[0] * 4; j = j + 1)//协方差
					{
						if (rows * row + j == rows * row + row)continue;
						if (abs(rows * row + j - (rows * row + row)) % 4 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaL * (DDCObs.DDSatNum[0] + 1));
					}
					row++; col++;
				}

			}
			else if (SDObs.SdSatObs[i].system == BDS)
			{
				if (!DDCObs.RefStatus[1]) continue;//没有选到基准星的大概率是单差观测值没有通过粗差检验
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.x) / PR[DDCObs.RefPos[1]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.y) / PR[DDCObs.RefPos[1]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.z) / PR[DDCObs.RefPos[1]];
				///每四行一次循环
				//第一二行
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dP[k]
						- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]]);
					arrayP[rows * row + row] = DDCObs.DDSatNum[1] / (2 * sigmaP * (DDCObs.DDSatNum[1] + 1));
					for (int j = DDCObs.DDSatNum[0] * 4; j < rows; j = j + 1)//协方差
					{
						if (rows * row + j == rows * row + row)continue;
						if (abs(rows * row + j - (rows * row + row)) % 4 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaP * (DDCObs.DDSatNum[1] + 1));
					}
					row++;
				}
				//第三四行
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayB[cols * row + 3 + col] = Cspeed / SDObs.SdSatObs[i].f[k];
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[1] / (2 * sigmaL * (DDCObs.DDSatNum[1] + 1));
					for (int j = DDCObs.DDSatNum[0] * 4; j < rows; j = j + 1)//协方差
					{
						if (rows * row + j == rows * row + row)continue;
						if (abs(rows * row + j - (rows * row + row)) % 4 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaL * (DDCObs.DDSatNum[1] + 1));
					}
					row++; col++;
				}

			}

		}
		//B
		Matrix B(rows, cols); B = arrayB;
		//B.Show();
		Matrix W(rows, 1); W = arrayW;
		//W.Show();
		Matrix P(rows, rows); P = arrayP;
		//P.Show();
		Matrix X(cols, 1); X = (B.Transpose() * P * B).Inverse() * B.Transpose() * P * W;
		//X.Show();
		normX = 0;
		for (int i = 0; i < cols; i++)
		{
			normX += pow(X(i, 0), 2);
		}
		normX = sqrt(normX);
		//更新
		//流动站位置
		roverPostion.x += X(0, 0);
		roverPostion.y += X(1, 0);
		roverPostion.z += X(2, 0);

		//模糊度更新
		for (int k = 0; k < cols - 3; k++)
			N[k] += X(k + 3, 0);

		Q = (B.Transpose() * P * B).Inverse(); 

		//Q.Show();
		Matrix V(rows, 1);V = B * X - W;
		//V.Show();
		rover.SigmaPos = ((V.Transpose() * P * V) * (1.0 / (rows - cols)))(0, 0);
		rover.SigmaPos = sqrt(rover.SigmaPos);//单位权中误差
		rover.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
		XYZ2BLH(rover._XYZpostion, rover._BLHpostion, a_GPS, alpha_GPS);
		double arrayH[9] = { -sin(deg2rad(rover._BLHpostion.b)) * cos(deg2rad(rover._BLHpostion.l)),-sin(deg2rad(rover._BLHpostion.b)) * sin(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.b)),
			-sin(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.l)),0,
			cos(deg2rad(rover._BLHpostion.b)) * cos(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.b)) * sin(deg2rad(rover._BLHpostion.l)),sin(deg2rad(rover._BLHpostion.b)) };
		Matrix H(3, 3); H = arrayH;
		double arrayQ[9] = { Q(0,0),Q(0,1) ,Q(0,2) ,Q(1,0), Q(1,1), Q(1,2) ,Q(2,0) ,Q(2,1),Q(2,2) };
		Matrix q(3, 3); q = arrayQ;
		Matrix _Q(3, 3); _Q = H * q * H.Transpose();
		rover.HDOP = sqrt(_Q(0, 0) + _Q(1, 1));
		rover.VDOP = sqrt(_Q(2, 2));

		count++;//计数器
		if (count > 20)
			break;
	}

	//基线向量
	DDCObs.dPos[0] = roverPostion.x - basePostion.x;
	DDCObs.dPos[1] = roverPostion.y - basePostion.y;
	DDCObs.dPos[2] = roverPostion.z - basePostion.z;
	//双频模糊度浮点解
	for (int k = 0; k < cols - 3; k++)
	{
		DDCObs.FloatAmb[k] = N[k];
		//printf("%lf\n", N[k]);
	}

	//模糊度协方差阵
	for (int k = 3; k < cols; k++)
	{
		for (int j = 3; j < cols; j++)
		{
			DDCObs.FloatAmbCov[(k - 3) * (cols - 3) + (j - 3)] = Q(k, j);
		}
	}
	rover._XYZpostion = roverPostion;//流动站位置重置
	//Matrix Qnn(cols - 3, cols - 3);
	//Qnn = DDCObs.FloatAmbCov;
	//Qnn.Show();

	////模糊度固定
	//if (lambda((DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]) * 2, 2, DDCObs.FloatAmb, DDCObs.FloatAmbCov, DDCObs.FixedAmb, DDCObs.ResAmb))
	//	return false;
	//DDCObs.Ratio = DDCObs.ResAmb[1] / DDCObs.ResAmb[0];
	//if (DDCObs.Ratio - 3 > EPS) DDCObs.bFixed = true;
	///*for (int i = 0; i < 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]); i++)
	//{
	//	DDCObs.FixedAmb[i] = (int)DDCObs.FixedAmb[i];
	//}*/

	////重新解算
	//Matrix amb(cols - 3, 1);//固定前
	//Matrix dpos(3, 1);
	//amb = N;
	//dpos = DDCObs.dPos;
	//Matrix ambFixed(cols - 3, 1);//固定后的模糊度
	//Matrix dposFixed(3, 1);
	//ambFixed = DDCObs.FixedAmb;
	//
	//Matrix Qba(3, cols - 3);
	//double* arrayQba = new double[(cols - 3) * 3];
	//for (int k = 0; k < 3; k++)
	//{
	//	for (int j = 3; j < cols; j++)
	//	{
	//		arrayQba[k * (cols - 3) + (j - 3)] = Q(k, j);
	//	}
	//}
	//Qba = arrayQba;
	//Matrix Qa(cols - 3, cols - 3);
	//Qa = DDCObs.FloatAmbCov;

	//dposFixed = dpos - Qba * Qa.Inverse() * (ambFixed - amb);
	//DDCObs.dPos[0] = dposFixed(0, 0);
	//DDCObs.dPos[1] = dposFixed(1, 0);
	//DDCObs.dPos[2] = dposFixed(2, 0);

	//rover._XYZpostion.x = basePostion.x + DDCObs.dPos[0];
	//rover._XYZpostion.y = basePostion.y + DDCObs.dPos[1];
	//rover._XYZpostion.z = basePostion.z + DDCObs.dPos[2];

	delete[] PR;
	delete[] PB;
	delete[] N;
	delete[] arrayB;
	delete[] arrayW;
	delete[] arrayP;
	//delete[] arrayQba;
	return true;
}

/// <summary>
/// 基站（流动站）到卫星的几何距离
/// </summary>
/// <param name="station">站坐标</param>
/// <param name="satPostion">卫星坐标数组</param>
/// <param name="index">索引</param>
/// <returns>几何距离</returns>
double distance(XYZ& station, SATPOS* satPostion, int index)
{
	return Norm(station, satPostion[index].XYZPos);
}

/// <summary>
/// 流动站固定解
/// </summary>
/// <param name="baseObs"></param>
/// <param name="roverObs"></param>
/// <param name="base"></param>
/// <param name="rover"></param>
/// <param name="SDObs"></param>
/// <param name="DDCObs"></param>
/// <returns></returns>
bool RTKFixed(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs)
{


	//1.设置基站和流动站初值
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion = rover._XYZpostion;
	//2.计算GPS和BDS,的双差卫星数
	//3/4计算站坐标到所有卫星的几何距离
	//按单差观测值数组的索引存，没有通过粗差探测的没有距离
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];

	//5.对单差观测值进行循环
	const int rows = 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int row = 0;//带入循环的变量
	const int cols = 3;
	int col = 0;//用来处理波长
	if (rows < cols) return false;//行数小于列数不予计算
	//模糊度初值
	double* N = new double[rows];//模糊度初值 //需进行赋值 //相位减伪距
	for (int i = 0; i < rows; i++)
	{
		N[i] = DDCObs.FixedAmb[i];
	}

	double normX = 1.0;
	int count = 0;
	double* arrayB = new double[rows * cols];//防止数组越界，多加一行//4*(GPS 双差 卫星数+BDS 双差 卫星数) 行 (3+2*(GPS 双差 卫星数+BDS 双差 卫星数))列
	for (int i = 0; i < rows * cols; i++)
		arrayB[i] = 0.0;
	double* arrayW = new double[rows];//w矩阵，亦或者w向量
	for (int i = 0; i < rows; i++)
		arrayW[i] = 0.0;
	double* arrayP = new double[rows * rows];//防止数组越界
	double sigmaL = 1;//单位都是m
	//sigmaL = pow(sigmaL, 2);
	for (int i = 0; i < rows * rows; i++)
		arrayP[i] = 0.0;
	//协方差阵
	Matrix Q(cols, cols);
	while (normX > 1E-8)
	{
		row = 0;
		col = 0;
		//计算站与星之间的距离
		//roverPostion = rover._XYZpostion;
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half != 1)continue;//有半周的不参与RTk
			PR[i] = distance(roverPostion, roverObs.satpos, SDObs.SdSatObs[i].nRov);
			PB[i] = distance(basePostion, baseObs.satpos, SDObs.SdSatObs[i].nBas);
		}

		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half != 1)continue;//有半周的不参与RTk
			if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//基准星不做差
			if (SDObs.SdSatObs[i].system == GPS)
			{
				if (!DDCObs.RefStatus[0]) continue;//没有选到基准星的大概率是单差观测值没有通过粗差检验
				//B矩阵
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.x) / PR[DDCObs.RefPos[0]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.y) / PR[DDCObs.RefPos[0]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.z) / PR[DDCObs.RefPos[0]];
				///每两行一次循环
				
				//第一二行
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[0] / (2 * sigmaL * (DDCObs.DDSatNum[0] + 1));
					for (int j = 0; j < DDCObs.DDSatNum[0] * 2; j = j + 1)//协方差
					{
						if (j == row)continue;
						if (abs(j - row) % 2 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaL * (DDCObs.DDSatNum[0] + 1));
					}
					row++; col++;
				}

			}
			else if (SDObs.SdSatObs[i].system == BDS)
			{
				if (!DDCObs.RefStatus[1]) continue;//没有选到基准星的大概率是单差观测值没有通过粗差检验
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.x) / PR[DDCObs.RefPos[1]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.y) / PR[DDCObs.RefPos[1]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.z) / PR[DDCObs.RefPos[1]];
				//第一二行
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[1] / (2 * sigmaL * (DDCObs.DDSatNum[1] + 1));
					for (int j = DDCObs.DDSatNum[0] * 2; j < rows; j ++)//协方差
					{
						if (j == row)continue;
						if (abs(j - row) % 2 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaL * (DDCObs.DDSatNum[1] + 1));
					}
					row++; col++;
				}

			}

		}
		//B
		Matrix B(rows, cols); B = arrayB;
		//B.Show();
		Matrix W(rows, 1); W = arrayW;
		//W.Show();
		Matrix P(rows, rows); P = arrayP;
		//P.Show();
		Matrix X(cols, 1); X = (B.Transpose() * P * B).Inverse() * B.Transpose() * P * W;
		//X.Show();
		normX = 0;
		for (int i = 0; i < cols; i++)
		{
			normX += pow(X(i, 0), 2);
		}
		normX = sqrt(normX);
		//更新
		//流动站位置
		roverPostion.x += X(0, 0);
		roverPostion.y += X(1, 0);
		roverPostion.z += X(2, 0);

		Q = (B.Transpose() * P * B).Inverse();


		Matrix V(rows, 1); V = B * X - W;
		//V.Show();
		rover.SigmaPos = ((V.Transpose() * P * V) * (1.0 / (rows - cols)))(0, 0);
		rover.SigmaPos = sqrt(rover.SigmaPos);//单位权中误差
		rover.PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
		XYZ2BLH(rover._XYZpostion, rover._BLHpostion, a_GPS, alpha_GPS);
		double arrayH[9] = { -sin(deg2rad(rover._BLHpostion.b)) * cos(deg2rad(rover._BLHpostion.l)),-sin(deg2rad(rover._BLHpostion.b)) * sin(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.b)),
			-sin(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.l)),0,
			cos(deg2rad(rover._BLHpostion.b)) * cos(deg2rad(rover._BLHpostion.l)),cos(deg2rad(rover._BLHpostion.b)) * sin(deg2rad(rover._BLHpostion.l)),sin(deg2rad(rover._BLHpostion.b)) };
		Matrix H(3, 3); H = arrayH;
		double arrayQ[9] = { Q(0,0),Q(0,1) ,Q(0,2) ,Q(1,0), Q(1,1), Q(1,2) ,Q(2,0) ,Q(2,1),Q(2,2) };
		Matrix q(3, 3); q = arrayQ;
		Matrix _Q(3, 3); _Q = H * q * H.Transpose();
		rover.HDOP = sqrt(_Q(0, 0) + _Q(1, 1));
		rover.VDOP = sqrt(_Q(2, 2));
		rover.mxyz[0] = rover.SigmaPos * sqrt(Q(0, 0));
		rover.mxyz[1] = rover.SigmaPos * sqrt(Q(1, 1));
		rover.mxyz[2] = rover.SigmaPos * sqrt(Q(2, 2));
		count++;//计数器
		if (count > 20)
			break;
	}

	//基线向量
	DDCObs.dPos[0] = roverPostion.x - basePostion.x;
	DDCObs.dPos[1] = roverPostion.y - basePostion.y;
	DDCObs.dPos[2] = roverPostion.z - basePostion.z;

	rover._XYZpostion = roverPostion;//流动站位置重置

	delete[] PR;
	delete[] PB;
	delete[] N;
	delete[] arrayB;
	delete[] arrayW;
	delete[] arrayP;
	return true;
}