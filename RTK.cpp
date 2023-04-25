#include"RTK.h"
/// <summary>
/// ʱ�����
/// ����վʱ��-��׼վʱ��<2s?
/// �ļ���ȡ��Ͼͷ���true
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
	//�ȶ�ȡ����վ�ļ�������
	//while��ȡrover����
	while (RoverStatus == 0)
		RoverStatus = _GetData(fp_rover, buffer_rover, n_rover, rtk.RoverObs, rtk);
	double delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;

	while (fabs(delta_time) - interval > EPS)
	{
		if (delta_time + interval < EPS)//��վ��ǰ������վ�ں��棬��ȡ����վ�����������㶨λ���
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

	//ʱ��ͬ���ɹ�����
	// ����0: ��ͬ��
	// ����1��ͬ��
	// ����2��ֻ������վ
	// ����-1���ļ���ȡ����
	//ʱ��ͬ���ɹ�����true�����ɹ��ͷ���false
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
/// �����봮�����ݶ���
/// </summary>
/// <param name="buffer_base"></param>
/// <param name="n_base"></param>
/// <param name="buffer_rover"></param>
/// <param name="n_rover"></param>
/// <param name="cse">����</param>
/// <param name="sock">����</param>
/// <param name="rtk"></param>
/// <returns></returns>
short _TimeSynchronization(unsigned char* buffer_base, int& n_base, unsigned char* buffer_rover, int& n_rover, CSerial& cse, SOCKET& sock, RTKData& rtk)
{
	short RoverStatus = 0;//ͨ�����ڻ�ȡ
	short BaseStatus = 0;//ͨ�������ȡ
	double interval = 0.1;//����������ʱ�����
	//�ȶ�����
	//�ȶ�ȡ����վ�ļ�������
	//while��ȡrover����
	while (RoverStatus == 0)
		RoverStatus = _GetData(cse, buffer_rover, n_rover, rtk.RoverObs, rtk);
	double delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
	while (fabs(delta_time) - interval > EPS)
	{
		if (delta_time + interval < EPS)//��վ��ǰ������վ�ں��棬��ȡ����վ�����������㶨λ���
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
	//ʱ��ͬ���ɹ�����
	// ����0: ��ͬ��
	// ����1��ͬ��
	// ����2��ֻ������վ
	// ����-1���ļ���ȡ����
	//ʱ��ͬ���ɹ�����true�����ɹ��ͷ���false
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
/// ʵʱʱ��ͬ��
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
	short RoverStatus = 0;//ͨ�����ڻ�ȡ
	short BaseStatus = 0;//ͨ�������ȡ
	double interval = 0;//����������ʱ�����
	//�ȶ�����
	//�ȶ�ȡ����վ�ļ�������
	//while��ȡrover����
	RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
	BaseStatus = _GetData(baseSock, buffer_base, n_base, rtk.BaseObs, rtk);
	//while (RoverStatus == 0)
	//	RoverStatus = _GetData(roverSock, buffer_rover, n_rover, rtk.RoverObs, rtk);
	double delta_time = (rtk.RoverObs.gt.Week - rtk.BaseObs.gt.Week) * 604800 + rtk.RoverObs.gt.SecOfWeek - rtk.BaseObs.gt.SecOfWeek;
	while (fabs(delta_time) > interval )
	{
		if (delta_time < -interval)//��վ��ǰ������վ�ں��棬��ȡ����վ�����������㶨λ���
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
	//ʱ��ͬ���ɹ�����
	// ����0: ��ͬ��
	// ����1��ͬ��
	// ����2��ֻ������վ
	// ����-1���ļ���ȡ����
	//ʱ��ͬ���ɹ�����true�����ɹ��ͷ���false
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
/// ���ݻ�ȡ
/// </summary>
/// <param name="buffer"></param>
/// <param name="n"></param>
/// <param name="obs"></param>
/// <param name="rtk"></param>
/// <param name="rover"></param>
/// <returns>1:��ȡ���ݳɹ�</returns>
/// <returns>0����ȡ���ݲ��ɹ�</returns>
/// <returns>-1���ļ���ȡ����</returns>
short _GetData(FILE* fp, unsigned char* buffer, int& n, OBS& obs, RTKData& rtk)
{
	//��ȡ����fread
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
/// վ�䵥�����վ����׼վ
/// </summary>
/// <param name="baseObs">��վ�۲�ֵ</param>
/// <param name="roverObs">����վ�۲�ֵ</param>
/// <param name="SDObs">����۲�ֵ</param>
void FormSDEpochObs(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs)
{
	SDSATOBS SDSatObs[MaxSatNum];
	memcpy(SDObs.SdSatObs, SDSatObs, sizeof(SDSATOBS) * MaxSatNum);
	//α��
	int num = 0;
	for (int i = 0; i < roverObs.SatNum; i++)
	{
		for (int j = 0; j < baseObs.SatNum; j++)
		{
			if (roverObs.satobs[i].system == baseObs.satobs[j].system && roverObs.satobs[i].PRN == baseObs.satobs[j].PRN)
			{
				if (fabs(roverObs.satobs[i].L[0]) < EPS || fabs(baseObs.satobs[j].L[0]) < EPS || fabs(roverObs.satobs[i].L[1]) < EPS || fabs(baseObs.satobs[j].L[1]) < EPS)
					break;//����������˫Ƶ�۲�ֵ����
				if (roverObs.satpos[i].Status == 1 && baseObs.satpos[j].Status == 1)//����λ�������ǹ۲�ֵ����һ��
					SDObs.SdSatObs[num].Valid = 0;//վ�䵥��������Ч
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
/// �ֲ�̽��
/// </summary>
/// <param name="SDObs"></param>
void DetectCycleSilp(SDEPOCHOBS& SDObs)
{
	MWGF outer[MaxSatNum];//����Ԫ��MW GF ���
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid == -1) continue;
		outer[i].system = SDObs.SdSatObs[i].system;
		outer[i].PRN = SDObs.SdSatObs[i].PRN;

		//�������Lmw��Lgf
		outer[i].Lmw = 1 / (SDObs.SdSatObs[i].f[0] - SDObs.SdSatObs[i].f[1]) * (SDObs.SdSatObs[i].f[0] * SDObs.SdSatObs[i].dL[0] - SDObs.SdSatObs[i].f[1] * SDObs.SdSatObs[i].dL[1])
			- 1.0 / (SDObs.SdSatObs[i].f[0] + SDObs.SdSatObs[i].f[1]) * (SDObs.SdSatObs[i].f[0] * SDObs.SdSatObs[i].dP[0] + SDObs.SdSatObs[i].f[1] * SDObs.SdSatObs[i].dP[1]);
		outer[i].Lgf = SDObs.SdSatObs[i].dL[0] - SDObs.SdSatObs[i].dL[1];

		//ͻȻ���ֵ�����Ľ��
		bool Status = false;
		//����һʱ�̵�PRN
		for (int j = 0; j < MaxSatNum; j++)
		{
			if (SDObs.mwgf[j].PRN == 0)continue;
			if (outer[i].system != SDObs.mwgf[j].system)continue;
			if (outer[i].PRN != SDObs.mwgf[j].PRN)continue;
			//�ҵ�j�� j ��һʱ��
			Status = true;
			//ǰ����Ԫ��_Lmw Lgf����
			double dGF = outer[i].Lgf - SDObs.mwgf[j].Lgf;
			double dMW = outer[i].Lmw - SDObs.mwgf[j]._Lmw;
			//����Ƿ���
			if (fabs(SDObs.mwgf[j]._Lmw) > EPS && fabs(dMW) <= MaxDeltaLmw && fabs(dGF) <= MaxDeltaLgf)
			{
				//���Ϊ����
				SDObs.SdSatObs[i].Valid = 1;//?������ôд��
			}
			else
			{
				//�ֲ���ּ����Ϊ2���ʱ�̵�ƽ��ֵΪ��������Lmw
				outer[i]._Lmw = outer[i].Lmw;
				outer[i].n = 1;
				SDObs.SdSatObs[i].Valid = 2;//˫Ƶ�۲�ֵ��������
				break;
			}
			//���㱾��Ԫ��_Lmw[1];
			outer[i]._Lmw = (double)(SDObs.mwgf[j].n - 1) / (double)SDObs.mwgf[j].n * SDObs.mwgf[j]._Lmw + 1.0 / (double)SDObs.mwgf[j].n * outer[i].Lmw;
			outer[i].n = SDObs.mwgf[j].n + 1;
			break;
		}
		if (!Status)
		{
			SDObs.SdSatObs[i].Valid = 3;//ͻȻ���ֵ�����
			outer[i]._Lmw = outer[i].Lmw;
			outer[i].n = 1;//��ʱ��Ϊ��һ����Ԫ
		}
	}
	memcpy(SDObs.mwgf, outer, MaxSatNum * sizeof(MWGF));
}

/// <summary>
/// �ο���ѡȡ
/// </summary>
/// <param name="baseObs"></param>
/// <param name="roverObs"></param>
/// <param name="SDObs"></param>
/// <param name="DDCObs"></param>
bool DetRefSat(const OBS& baseObs, const OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs)
{
	//�ο���ѡȡ
	//��һʱ�̵Ĳο���
	int RefPrn[2] = { DDCObs.RefPrn[0] ,DDCObs.RefPrn[1] };
	DDCObs = DDCOBS();
	double CN0L1[2], CN0L2[2];
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		else DDCObs.Sats++; //0.ͨ���ֲ�̽������˫Ƶ�۲�ֵ�����Լ�������λ��
		if (SDObs.SdSatObs[i].system == GPS) {
			

			if (baseObs.satobs[SDObs.SdSatObs[i].nBas].ParityKnownFlag[0] == 0 || baseObs.satobs[SDObs.SdSatObs[i].nBas].ParityKnownFlag[1] == 0) continue;
			else if (roverObs.satobs[SDObs.SdSatObs[i].nRov].ParityKnownFlag[0] == 0 || roverObs.satobs[SDObs.SdSatObs[i].nRov].ParityKnownFlag[1] == 0)continue;
			else
				SDObs.SdSatObs[i].half = 1;//2.������û�а���

			DDCObs.DDSatNum[0]++;//GPS ����˫��۲�ֵ��������

			//1.��һ��Ԫ��Ϊ�ο��ǿ�ֱ�ӳ�Ϊ����Ԫ�ο���
			if (RefPrn[0] == SDObs.SdSatObs[i].PRN&&RefPrn[0]>0) {
				DDCObs.RefPrn[0] = SDObs.SdSatObs[i].PRN;
				DDCObs.RefPos[0] = i;
				DDCObs.RefStatus[0] = 1; 
				continue;
			}

			if (DDCObs.RefStatus[0] == 1)continue;//���������ɹ��Ļ���ѡ������Ϊ�ο���


			if (baseObs.satobs[SDObs.SdSatObs[i].nBas].CNR[0] - 40.0 < EPS || baseObs.satobs[SDObs.SdSatObs[i].nBas].CNR[1] - 28.0 < EPS) continue;
			else if (roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[0] - 40.0 < EPS || roverObs.satobs[SDObs.SdSatObs[i].nRov].CNR[1] - 28.0 < EPS) continue;
			else
			{
				//3.����CN0�ж�
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
				SDObs.SdSatObs[i].half = 1;//2.������û�а���

			DDCObs.DDSatNum[1]++;//BDS ����˫��۲�ֵ��������

			//1.��һ��Ԫ��Ϊ�ο��ǿ�ֱ�ӳ�Ϊ����Ԫ�ο���
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
	//û���ҵ�������ȵĻ�׼��
	//��ʱ����bugֻ���ڲ���
	CN0L1[0] = CN0L1[1] = CN0L2[0] = CN0L2[1] = 0.0;
	if (DDCObs.RefPos[0] < 0)
	{
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;//0.ͨ���ֲ�̽������˫Ƶ�۲�ֵ�����Լ�������λ��
			if (SDObs.SdSatObs[i].half == 0)continue;//2.�а��ܵĲ�ѡ
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
			if (SDObs.SdSatObs[i].Valid != 1)continue;//0.ͨ���ֲ�̽������˫Ƶ�۲�ֵ�����Լ�������λ��
			if (SDObs.SdSatObs[i].half == 0)continue;//2.�а��ܵĲ�ѡ
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
	//�ҵ�֮��������ǣ����ÿ�ζ���ˢ��
	if (DDCObs.RefPos[0] >= 0) { DDCObs.RefStatus[0] = 1; DDCObs.DDSatNum[0]--; }
	if (DDCObs.RefPos[1] >= 0) { DDCObs.RefStatus[1] = 1; DDCObs.DDSatNum[1]--; }
	if (DDCObs.RefPos[0] == -1 && DDCObs.RefPos[1] == -1) return false;
	else return true;

}

/// <summary>
/// ��Զ�λ�����
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
	//1.���û�վ������վ��ֵ
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion = rover._XYZpostion;
	//2.����GPS��BDS��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];

	//5.�Ե���۲�ֵ����ѭ��
	const int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int row = 0;//����ѭ���ı���
	const int cols = 3 + 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int col = 0;//����������
	if (rows < cols) return false;//����С�������������
	//ģ���ȳ�ֵ
	double* N = new double[cols - 3];//ģ���ȳ�ֵ //����и�ֵ //��λ��α��
	int Ncol = 0;
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1])continue;
		//!!!!!!˫����λ��˫��α��  ���Բ���
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
	double* arrayB = new double[rows * cols];//��ֹ����Խ�磬���һ��//4*(GPS ˫�� ������+BDS ˫�� ������) �� (3+2*(GPS ˫�� ������+BDS ˫�� ������))��
	for (int i = 0; i < rows * cols; i++)
		arrayB[i] = 0.0;
	double* arrayW = new double[rows];//w���������w����
	for (int i = 0; i < rows; i++)
		arrayW[i] = 0.0;
	double* arrayP = new double[rows * rows];//��ֹ����Խ��
	double sigmaP = 0.5;
	double sigmaL = 0.0005;//��λ����m
	//sigmaP = pow(sigmaP, 2);
	//sigmaL = pow(sigmaL, 2);
	for (int i = 0; i < rows * rows ; i++)
		arrayP[i] = 0.0;
	//Э������
	Matrix Q(cols, cols);
	while (normX > 1E-7)
	{
		row = 0;
		col = 0;
		//����վ����֮��ľ���
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
			PR[i] = distance(roverPostion, roverObs.satpos, SDObs.SdSatObs[i].nRov);
			PB[i] = distance(basePostion, baseObs.satpos, SDObs.SdSatObs[i].nBas);
		}
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half !=1)continue;//�а��ܵĲ�����RTk
			if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
			if (SDObs.SdSatObs[i].system == GPS)
			{
				if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
				//B����
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.x) / PR[DDCObs.RefPos[0]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.y) / PR[DDCObs.RefPos[0]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.z) / PR[DDCObs.RefPos[0]];
				///ÿ����һ��ѭ��
				//��һ������
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dP[k]
						- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]]);
					arrayP[rows * row + row] = DDCObs.DDSatNum[0] / (2 * sigmaP * (DDCObs.DDSatNum[0] + 1));//���Խ��ߣ�������
					for (int j = 0; j < DDCObs.DDSatNum[0] * 4; j = j + 1)//Э����
					{
						if (rows * row + j == rows * row + row)continue;
						if (abs(rows * row + j - (rows * row + row)) % 4 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaP * (DDCObs.DDSatNum[0] + 1));
					}
					row++;
				}
				//����������
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayB[cols * row + 3 + col] = Cspeed / SDObs.SdSatObs[i].f[k];
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[0] / (2 * sigmaL * (DDCObs.DDSatNum[0] + 1));
					for (int j = 0; j < DDCObs.DDSatNum[0] * 4; j = j + 1)//Э����
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
				if (!DDCObs.RefStatus[1]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.x) / PR[DDCObs.RefPos[1]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.y) / PR[DDCObs.RefPos[1]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.z) / PR[DDCObs.RefPos[1]];
				///ÿ����һ��ѭ��
				//��һ����
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dP[k]
						- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]]);
					arrayP[rows * row + row] = DDCObs.DDSatNum[1] / (2 * sigmaP * (DDCObs.DDSatNum[1] + 1));
					for (int j = DDCObs.DDSatNum[0] * 4; j < rows; j = j + 1)//Э����
					{
						if (rows * row + j == rows * row + row)continue;
						if (abs(rows * row + j - (rows * row + row)) % 4 > 0)continue;
						arrayP[rows * row + j] = -1 / (2 * sigmaP * (DDCObs.DDSatNum[1] + 1));
					}
					row++;
				}
				//��������
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayB[cols * row + 3 + col] = Cspeed / SDObs.SdSatObs[i].f[k];
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[1] / (2 * sigmaL * (DDCObs.DDSatNum[1] + 1));
					for (int j = DDCObs.DDSatNum[0] * 4; j < rows; j = j + 1)//Э����
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
		//����
		//����վλ��
		roverPostion.x += X(0, 0);
		roverPostion.y += X(1, 0);
		roverPostion.z += X(2, 0);

		//ģ���ȸ���
		for (int k = 0; k < cols - 3; k++)
			N[k] += X(k + 3, 0);

		Q = (B.Transpose() * P * B).Inverse(); 

		//Q.Show();
		Matrix V(rows, 1);V = B * X - W;
		//V.Show();
		rover.SigmaPos = ((V.Transpose() * P * V) * (1.0 / (rows - cols)))(0, 0);
		rover.SigmaPos = sqrt(rover.SigmaPos);//��λȨ�����
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

		count++;//������
		if (count > 20)
			break;
	}

	//��������
	DDCObs.dPos[0] = roverPostion.x - basePostion.x;
	DDCObs.dPos[1] = roverPostion.y - basePostion.y;
	DDCObs.dPos[2] = roverPostion.z - basePostion.z;
	//˫Ƶģ���ȸ����
	for (int k = 0; k < cols - 3; k++)
	{
		DDCObs.FloatAmb[k] = N[k];
		//printf("%lf\n", N[k]);
	}

	//ģ����Э������
	for (int k = 3; k < cols; k++)
	{
		for (int j = 3; j < cols; j++)
		{
			DDCObs.FloatAmbCov[(k - 3) * (cols - 3) + (j - 3)] = Q(k, j);
		}
	}
	rover._XYZpostion = roverPostion;//����վλ������
	//Matrix Qnn(cols - 3, cols - 3);
	//Qnn = DDCObs.FloatAmbCov;
	//Qnn.Show();

	////ģ���ȹ̶�
	//if (lambda((DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]) * 2, 2, DDCObs.FloatAmb, DDCObs.FloatAmbCov, DDCObs.FixedAmb, DDCObs.ResAmb))
	//	return false;
	//DDCObs.Ratio = DDCObs.ResAmb[1] / DDCObs.ResAmb[0];
	//if (DDCObs.Ratio - 3 > EPS) DDCObs.bFixed = true;
	///*for (int i = 0; i < 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]); i++)
	//{
	//	DDCObs.FixedAmb[i] = (int)DDCObs.FixedAmb[i];
	//}*/

	////���½���
	//Matrix amb(cols - 3, 1);//�̶�ǰ
	//Matrix dpos(3, 1);
	//amb = N;
	//dpos = DDCObs.dPos;
	//Matrix ambFixed(cols - 3, 1);//�̶����ģ����
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
/// ��վ������վ�������ǵļ��ξ���
/// </summary>
/// <param name="station">վ����</param>
/// <param name="satPostion">������������</param>
/// <param name="index">����</param>
/// <returns>���ξ���</returns>
double distance(XYZ& station, SATPOS* satPostion, int index)
{
	return Norm(station, satPostion[index].XYZPos);
}

/// <summary>
/// ����վ�̶���
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


	//1.���û�վ������վ��ֵ
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion = rover._XYZpostion;
	//2.����GPS��BDS,��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];

	//5.�Ե���۲�ֵ����ѭ��
	const int rows = 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int row = 0;//����ѭ���ı���
	const int cols = 3;
	int col = 0;//����������
	if (rows < cols) return false;//����С�������������
	//ģ���ȳ�ֵ
	double* N = new double[rows];//ģ���ȳ�ֵ //����и�ֵ //��λ��α��
	for (int i = 0; i < rows; i++)
	{
		N[i] = DDCObs.FixedAmb[i];
	}

	double normX = 1.0;
	int count = 0;
	double* arrayB = new double[rows * cols];//��ֹ����Խ�磬���һ��//4*(GPS ˫�� ������+BDS ˫�� ������) �� (3+2*(GPS ˫�� ������+BDS ˫�� ������))��
	for (int i = 0; i < rows * cols; i++)
		arrayB[i] = 0.0;
	double* arrayW = new double[rows];//w���������w����
	for (int i = 0; i < rows; i++)
		arrayW[i] = 0.0;
	double* arrayP = new double[rows * rows];//��ֹ����Խ��
	double sigmaL = 1;//��λ����m
	//sigmaL = pow(sigmaL, 2);
	for (int i = 0; i < rows * rows; i++)
		arrayP[i] = 0.0;
	//Э������
	Matrix Q(cols, cols);
	while (normX > 1E-8)
	{
		row = 0;
		col = 0;
		//����վ����֮��ľ���
		//roverPostion = rover._XYZpostion;
		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
			PR[i] = distance(roverPostion, roverObs.satpos, SDObs.SdSatObs[i].nRov);
			PB[i] = distance(basePostion, baseObs.satpos, SDObs.SdSatObs[i].nBas);
		}

		for (int i = 0; i < SDObs.SatNum; i++)
		{
			if (SDObs.SdSatObs[i].Valid != 1)continue;
			if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
			if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
			if (SDObs.SdSatObs[i].system == GPS)
			{
				if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
				//B����
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.x) / PR[DDCObs.RefPos[0]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.y) / PR[DDCObs.RefPos[0]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.z) / PR[DDCObs.RefPos[0]];
				///ÿ����һ��ѭ��
				
				//��һ����
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[0] / (2 * sigmaL * (DDCObs.DDSatNum[0] + 1));
					for (int j = 0; j < DDCObs.DDSatNum[0] * 2; j = j + 1)//Э����
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
				if (!DDCObs.RefStatus[1]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
				double l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
					- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.x) / PR[DDCObs.RefPos[1]];
				double m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
					- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.y) / PR[DDCObs.RefPos[1]];
				double n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
					- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.z) / PR[DDCObs.RefPos[1]];
				//��һ����
				for (int k = 0; k < 2; k++)
				{
					arrayB[cols * row] = l; arrayB[cols * row + 1] = m; arrayB[cols * row + 2] = n;
					arrayW[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
						- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
						- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
					arrayP[rows * row + row] = DDCObs.DDSatNum[1] / (2 * sigmaL * (DDCObs.DDSatNum[1] + 1));
					for (int j = DDCObs.DDSatNum[0] * 2; j < rows; j ++)//Э����
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
		//����
		//����վλ��
		roverPostion.x += X(0, 0);
		roverPostion.y += X(1, 0);
		roverPostion.z += X(2, 0);

		Q = (B.Transpose() * P * B).Inverse();


		Matrix V(rows, 1); V = B * X - W;
		//V.Show();
		rover.SigmaPos = ((V.Transpose() * P * V) * (1.0 / (rows - cols)))(0, 0);
		rover.SigmaPos = sqrt(rover.SigmaPos);//��λȨ�����
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
		count++;//������
		if (count > 20)
			break;
	}

	//��������
	DDCObs.dPos[0] = roverPostion.x - basePostion.x;
	DDCObs.dPos[1] = roverPostion.y - basePostion.y;
	DDCObs.dPos[2] = roverPostion.z - basePostion.z;

	rover._XYZpostion = roverPostion;//����վλ������

	delete[] PR;
	delete[] PB;
	delete[] N;
	delete[] arrayB;
	delete[] arrayW;
	delete[] arrayP;
	return true;
}