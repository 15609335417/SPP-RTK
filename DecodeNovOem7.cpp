#include"Decode.h"

/// <summary>
/// CRC校验
/// </summary>
/// <param name="buff"></param>
/// <param name="len"></param>
/// <returns></returns>
unsigned int crc32(const unsigned char* buff, int len)
{
	int i, j;
	unsigned int crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buff[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
			else crc >>= 1;
		}
	}
	return crc;
}
//double类型的数据解码
double D8(unsigned char* p)
{
	double r;
	memcpy(&r, p, 8);
	return r;
}
float F4(unsigned char* p)
{
	float r;
	memcpy(&r, p, 4);
	return r;
}
//usigned short类型数据解码
unsigned short US2(unsigned char* p)
{
	unsigned short r;
	memcpy(&r, p, 2);
	return r;
}
short S2(unsigned char* p)
{
	short r;
	memcpy(&r, p, 2);
	return r;
}
unsigned int UI4(unsigned char* p)
{
	unsigned int r;
	memcpy(&r, p, 4);
	return r;
}
long L4(unsigned char* p)
{
	long r;
	memcpy(&r, p, 4);
	return r;
}
unsigned long UL4(unsigned char* p)
{
	unsigned long r;
	memcpy(&r, p, 4);
	return r;
}
template<typename OutType, typename InType, size_t Cut_Count>
OutType BitCut(InType ul, int startIndex)
{
	return static_cast<OutType>(((ul >> startIndex) & (~(~0 << Cut_Count))));
}

/// <summary>
/// Novoem7板卡解码
/// </summary>
/// <param name="buffer">缓冲区</param>
/// <param name="n">剩余字节数</param>
/// <param name="obs">观测值</param>
/// <param name="eph">星历</param>
/// <param name="basesta">基站位置</param>
/// <returns>观测值状态</returns>
bool _DecodeNovOem7(unsigned char* buffer, int& n, OBS& obs, EPH* eph, BASESTATION& basesta)
{
	uint16_t MessageID = 0;
	uint16_t Message_Length = 0;
	int i = 0, j = 0;
	GPST gt;
	bool Status = false;
	int BytENUm = (cfg.mode == 1|| cfg.mode ==2) ? n : MaxNovNum;//方便串口以后加进来，还没debug
	for (i = 0; i < BytENUm; i++)
	{
		if (!(buffer[i] == 0xAA && buffer[i + 1] == 0x44 && buffer[i + 2] == 0x12))continue;
		if (i + 28 >= BytENUm)break;
		//MessageID
		MessageID = US2(buffer + i + 4);
		Message_Length = US2(buffer + i + 8);
		//获取GPS周，周内秒 ，即当前定位时刻
		gt.Week = US2(buffer + i + 14);
		gt.SecOfWeek = (double)(UI4(buffer + i + 16) * 1.0e-3);
		if (i + 3 + 25 + Message_Length + 4 >= BytENUm) break;
		if (crc32(buffer + i, 28 + Message_Length) != UI4(buffer + i + 3 + 25 + Message_Length))
		{
			i += Message_Length + 32;
			continue;
		}
		switch (MessageID)
		{
		case 7:
			_DecodeEphG(buffer, i, eph);
			break;
		case 42:
			DecodeBase(buffer, i, basesta);
			//if (obs.satobs->PRN != 0)Status = true;
			//if (cfg.mode == 0)
			//{
			//	n = 0;
			//	for (i = i + Message_Length + 32; i < BytENUm; i++) {
			//		buffer[n] = buffer[i];
			//		n++;
			//	}
			//	return Status;
			//}

			break;
		case 43:
			DecodeRange(buffer, i, obs);
			if (obs.satobs->PRN != 0)Status = true;
			if (cfg.mode == 0)
			{
				n = 0;
				for (i = i + Message_Length + 32; i < BytENUm; i++) {
					buffer[n] = buffer[i];
					n++;
				}
				return Status;
			}
			break;
		case 1696:
			_DecodeEphC(buffer, i, eph);
			break;
		default:
			break;
		}
		i += Message_Length + 32 - 1;
	}
	
	n = 0;
	for (; i < BytENUm; i++)
	{
		buffer[n] = buffer[i];
		n++;
	}
	return Status;
}

///星历解码，将所有的星历都放在同一个卫星星历结构体数组下
/// <summary>
/// GPS星历解码
/// </summary>
/// <param name="buffer"></param>
/// <param name="i"></param>
/// <param name="eph"></param>
void _DecodeEphG(unsigned char* buffer, int i, EPH* eph)
{
	unsigned short PRN = 0;
	PRN = (short)UL4(buffer + i + 28);
	if (PRN < 0 || PRN>MaxGPSSatNum)return;
	(eph + PRN - 1)->system = GPS;
	(eph + PRN - 1)->PRN = PRN;
	(eph + PRN - 1)->gt.Week = US2(buffer + i + 14);
	(eph + PRN - 1)->gt.SecOfWeek = (double)(UI4(buffer + i + 16) * 1.0e-3);
	i += 28;
	(eph + PRN - 1)->Health = (short)UL4(buffer + i + 12);
	(eph + PRN - 1)->IODE[0] = UL4(buffer + i + 16);
	(eph + PRN - 1)->IODE[1] = UL4(buffer + i + 20);
	(eph + PRN - 1)->toe.Week = (short)UL4(buffer + i + 24);
	(eph + PRN - 1)->toe.SecOfWeek = D8(buffer + i + 32);
	(eph + PRN - 1)->A = D8(buffer + i + 40);
	(eph + PRN - 1)->Deltan = D8(buffer + i + 48);
	(eph + PRN - 1)->M0 = D8(buffer + i + 56);
	(eph + PRN - 1)->e = D8(buffer + i + 64);
	(eph + PRN - 1)->omega = D8(buffer + i + 72);
	(eph + PRN - 1)->Cuc = D8(buffer + i + 80);
	(eph + PRN - 1)->Cus = D8(buffer + i + 88);
	(eph + PRN - 1)->Crc = D8(buffer + i + 96);
	(eph + PRN - 1)->Crs = D8(buffer + i + 104);
	(eph + PRN - 1)->Cic = D8(buffer + i + 112);
	(eph + PRN - 1)->Cis = D8(buffer + i + 120);
	(eph + PRN - 1)->i0 = D8(buffer + i + 128);
	(eph + PRN - 1)->Hati = D8(buffer + i + 136);
	(eph + PRN - 1)->OMEGA0 = D8(buffer + i + 144);
	(eph + PRN - 1)->HatOMEGA = D8(buffer + i + 152);
	(eph + PRN - 1)->toc = D8(buffer + i + 164);
	(eph + PRN - 1)->TGD[0] = D8(buffer + i + 172);
	(eph + PRN - 1)->a[0] = D8(buffer + i + 180);
	(eph + PRN - 1)->a[1] = D8(buffer + i + 188);
	(eph + PRN - 1)->a[2] = D8(buffer + i + 196);
}

/// <summary>
/// BDS 星历解码
/// </summary>
/// <param name="buffer"></param>
/// <param name="i"></param>
/// <param name="eph"></param>
void _DecodeEphC(unsigned char* buffer, int i, EPH* eph)
{
	unsigned short PRN = 0;
	PRN = (short)UL4(buffer + i + 28) + MaxGPSSatNum;
	if (PRN < MaxGPSSatNum || PRN>MaxBDSSatNum + MaxGPSSatNum)return;
	(eph + PRN - 1)->PRN = (short)UL4(buffer + i + 28);
	(eph + PRN - 1)->system = BDS;
	(eph + PRN - 1)->gt.Week = US2(buffer + i + 14);
	(eph + PRN - 1)->gt.SecOfWeek = (double)(UI4(buffer + i + 16) * 1.0e-3);
	i += 28;
	(eph + PRN - 1)->PRN = (short)UL4(buffer + i);
	(eph + PRN - 1)->toe.Week = (short)UL4(buffer + i + 4) + 1356;
	(eph + PRN - 1)->Health = (short)UL4(buffer + i + 16);
	(eph + PRN - 1)->TGD[0] = D8(buffer + i + 20);
	(eph + PRN - 1)->TGD[1] = D8(buffer + i + 28);
	(eph + PRN - 1)->toc = (double)UL4(buffer + i + 40);
	(eph + PRN - 1)->a[0] = D8(buffer + i + 44);
	(eph + PRN - 1)->a[1] = D8(buffer + i + 52);
	(eph + PRN - 1)->a[2] = D8(buffer + i + 60);
	(eph + PRN - 1)->toe.SecOfWeek = (double)UL4(buffer + i + 72);
	(eph + PRN - 1)->A = D8(buffer + i + 76) * D8(buffer + i + 76);
	(eph + PRN - 1)->e = D8(buffer + i + 84);
	(eph + PRN - 1)->omega = D8(buffer + i + 92);
	(eph + PRN - 1)->Deltan = D8(buffer + i + 100);
	(eph + PRN - 1)->M0 = D8(buffer + i + 108);
	(eph + PRN - 1)->OMEGA0 = D8(buffer + i + 116);
	(eph + PRN - 1)->HatOMEGA = D8(buffer + i + 124);
	(eph + PRN - 1)->i0 = D8(buffer + i + 132);
	(eph + PRN - 1)->Hati = D8(buffer + i + 140);
	(eph + PRN - 1)->Cuc = D8(buffer + i + 148);
	(eph + PRN - 1)->Cus = D8(buffer + i + 156);
	(eph + PRN - 1)->Crc = D8(buffer + i + 164);
	(eph + PRN - 1)->Crs = D8(buffer + i + 172);
	(eph + PRN - 1)->Cic = D8(buffer + i + 180);
	(eph + PRN - 1)->Cis = D8(buffer + i + 188);
}

/// <summary>
/// Range解码
/// </summary>
/// <param name="buffer">缓冲区</param>
/// <param name="i">偏移量</param>
/// <param name="obs">观测值结构体</param>
void DecodeRange(unsigned char* buffer, int i, OBS& obs)
{
	SatObs* satobs = new SatObs[MaxSatNum];//北斗48，GPS32
	SATPOS* satpos = new SATPOS[MaxSatNum];//中间结果
	double frequence[2];
	NAVSYS system;
	uint16_t Message_Length = US2(buffer + i + 8);
	obs.gt.Week = US2(buffer + i + 14);
	obs.gt.SecOfWeek = (double)(UI4(buffer + i + 16) * 1.0e-3);//GPS
	unsigned long track;
	unsigned long ch_tr_status;
	unsigned short prn = 0;
	i += 28;
	obs.ObsNum = (short)UL4(buffer + i);//观测值数量
	int k = 0;//Satnum
	int f = 0;//0 1 2
	obs.GPSNum = 0;
	obs.BDSNum = 0;
	for (int j = 0; j < obs.ObsNum; j++)
	{
		track = UL4(buffer + i + 44);
		switch ((track >> 16) & 7)
		{
		case 0:
			system = GPS;
			frequence[0] = L1C;//单位是Hz   L1C/A
			frequence[1] = L2P;//L2P
			switch ((track >> 21) & 0x1F)
			{
			case 0:f = 0; obs.GPSNum++; break;
			case 9:f = 1; obs.GPSNum++; break;
			default:f = 2; break;
			}
			break;
		case 1:system = GLONASS; f = 2; break;
		case 2:system = SBAS; f = 2; break;
		case 3:system = Galileo; f = 2; break;
		case 4:
			system = BDS;
			frequence[0] = B1I;//单位是Hz  B1I
			frequence[1] = B3I;//B3I
			switch ((track >> 21) & 0x1F)
			{
			case 0:f = 0; obs.BDSNum++; break;
			case 2:f = 1; obs.BDSNum++; break;
			case 4:f = 0; obs.BDSNum++; break;
			case 6:f = 1; obs.BDSNum++; break;
			default:f = 2; break;
			}
			break;
		case 5:system = QZSS; f = 2; break;
		case 6:system = NavIC; f = 2; break;
		case 7:system = Other; f = 2; break;
		default:f = 2;
			break;
		}

		if (f == 2)
		{
			i += 44;
			continue;
		}

		prn = US2(buffer + i + 4);
		for (int j = 0; j < k; j++)
		{
			if (satobs[j].system == system && satobs[j].PRN == prn)
			{
				k = j;
				if (system == GPS)
					obs.GPSNum--;
				else if (system == BDS)
					obs.BDSNum--;
				break;
			}
		}
		satobs[k].system = system;
		satpos[k].system = system;
		satobs[k].PRN = prn;
		satpos[k].PRN = prn;//中间结果
		satobs[k].f[f] = frequence[f];
		satobs[k].CNR[f] = F4(buffer + i + 36);
		satobs[k].C[f] = D8(buffer + i + 8);
		satobs[k].L[f] = (-D8(buffer + i + 20) * (Cspeed / satobs[k].f[f]));
		satobs[k].D[f] = (double)(-F4(buffer + i + 32) * (Cspeed / satobs[k].f[f]));
		satobs[k].PsrSigma[f] = F4(buffer + i + 16);
		satobs[k].AdrSigma[f] = F4(buffer + i + 28);
		satobs[k].locktime[f] = F4(buffer + i + 40);
		ch_tr_status = UL4(buffer + i + 44);
		satobs[k].PhaseLockFlag[f] = BitCut<short, unsigned long, 1>(ch_tr_status, 10);
		satobs[k].ParityKnownFlag[f] = BitCut<short, unsigned long, 1>(ch_tr_status, 11);
		satobs[k].CodeLockedFlag[f] = BitCut<short, unsigned long, 1>(ch_tr_status, 12);
		satobs[k].CarrierPhaseMeasurement[f] = BitCut<short, unsigned long, 1>(ch_tr_status, 28);

		k++;
		i += 44;
	}
	obs.SatNum = k;
	memcpy(obs.satobs, satobs, MaxSatNum * sizeof(SatObs));
	memcpy(obs.satpos, satpos, MaxSatNum * sizeof(SATPOS));
	delete[] satobs;
	delete[] satpos;
}
/// <summary>
/// BESTPOS解码
/// </summary>
/// <param name="buffer">缓冲区</param>
/// <param name="i">偏移量</param>
/// <param name="base">基站位置结构体</param>
void DecodeBase(unsigned char* buffer, int i, BASESTATION& base)
{
	base.gt.Week = US2(buffer + i + 14);
	base.gt.SecOfWeek = (double)(UI4(buffer + i + 16) * 1.0e-3);
	i += 28;
	base.BLHBESTPOS.b = D8(buffer + i + 8);
	base.BLHBESTPOS.l = D8(buffer + i + 16);
	base.BLHBESTPOS.h = D8(buffer + i + 24) + F4(buffer + i + 32);
	base.PosAccuracy[0] = F4(buffer + i + 40);//米
	base.PosAccuracy[1] = F4(buffer + i + 44);
	base.PosAccuracy[2] = F4(buffer + i + 48);
	BLH2XYZ(base.BLHBESTPOS, base.XYZBESTPOS, a_GPS, alpha_GPS);

}
