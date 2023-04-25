#include"OutFile.h"
void _SetOutfileName()
{
	////获取当地时间作为文件名
			///*获取当地时间*/
			//char date[16];
			//time_t timep;
			//time(&timep);
			//strftime(date, sizeof(date), "%Y%m%d%H%M", localtime(&timep));

			//char father[] = "data\\";
			//char hex[] = ".ome719";
			//char log[] = ".oem719.log";
			//char error[] = ".oem719.error";
			//char LogFileName[256];
			//sprintf(LogFileName, "%s%s%s", father, date, log);
			//char ErrorFileName[256];
			//sprintf(ErrorFileName, "%s%s%s", father, date, error);
			//char WriteFileName[256];
			//sprintf(WriteFileName, "%s%s%s", father, date, hex);

			////ft.open(LogFileName);
			////fv.open(ErrorFileName);

			////fopen_s(&fp, WriteFileName, "w+b");
}
/// <summary>
/// 定位结果输出
/// </summary>
/// <param name="ft">输出的文件</param>
/// <param name="gt">定位时间</param>
/// <param name="obs">观测值</param>
/// <param name="postion">定位结果</param>
void _OUTPostionFile(std::ofstream& ft, rover&rover)
{
	ft << "SPP:" << rover.gt.Week << ' ' << std::fixed << std::setprecision(3) << rover.gt.SecOfWeek << ' ';
	ft << "X:" << std::setw(13) << std::fixed << std::setprecision(4) << rover._XYZpostion.x << ' ';
	ft << "Y:" << std::setw(13) << rover._XYZpostion.y << ' ';
	ft << "Z:" << std::setw(13) << rover._XYZpostion.z << ' ';
	ft << "VX:" << std::setw(13) << std::fixed << std::setprecision(4) << rover.Velocity[0] << ' ';
	ft << "VY:" << std::setw(13) << rover.Velocity[1] << ' ';
	ft << "VZ:" << std::setw(13) << rover.Velocity[2] << ' ';
	ft << "B:" << std::setw(13) << std::setprecision(8) << rover._BLHpostion.b << ' ';
	ft << "L:" << std::setw(13) << rover._BLHpostion.l << ' ';
	ft << "H:" << std::setw(13) << std::setprecision(3) << rover._BLHpostion.h << ' ';
	ft << "GPS Clk:" << std::setw(12) << rover.RcvClkOft[0] << ' ';
	ft << "BDS Clk:" << std::setw(12) << rover.RcvClkOft[1] << ' ';
	ft << "PDOP:" << std::setw(8) << rover.PDOP << ' ';
	ft << "HDOP:" << std::setw(8) << rover.HDOP << ' ';
	ft << "VDOP:" << std::setw(8) << rover.VDOP << ' ';
	ft << "SigmaPos:" << std::setw(8) << rover.SigmaPos << ' ';
	ft << "Clkd=" << std::setw(12) << rover.RcvClkSft << ' ';
	//ft << "VDOT:" << std::setw(8) << postion.VDOT << ' ';
	ft << "SigmaVel:" << std::setw(8) << rover.SigmaVel << ' ';
	ft << "GPSSats:" << std::dec << std::setw(3) << rover.GPSSatNum << ' ';
	ft << "BDSSats:" << std::setw(3) << rover.BDSSatNum << ' ';
	ft << "Sats:" << std::setw(3) << rover.AllSatNum << '\n';

}
/// <summary>
/// 卫星位置输出
/// </summary>
/// <param name="ft">输出文件</param>
/// <param name="obs">卫星位置</param>
void _OUTSatPostionFile(std::ofstream& ft, OBS& obs)
{
	for (int t = 0; t < MaxSatNum; t++)
	{
		if (obs.satpos[t].Status == 0) continue;
		if (obs.satpos[t].Status == 2) continue;
		if (obs.satpos[t].Status == 3) continue;
		//if (fabs(obs.satpos[t].XYZPos.x) < EPS); continue;
		if (obs.satpos[t].system == GPS)
			ft << 'G' << std::setw(2) << std::setfill('0') << obs.satpos[t].PRN << ' ';
		else if (obs.satpos[t].system == BDS)
			ft << 'C' << std::setw(2) << std::setfill('0') << obs.satpos[t].PRN << ' ';
		ft << std::setfill(' ');
		ft << "X=" << std::setw(13) << std::fixed << std::setprecision(3) << obs.satpos[t].XYZPos.x << ' ';
		ft << "Y=" << std::setw(13) << obs.satpos[t].XYZPos.y << ' ';
		ft << "Z=" << std::setw(13) << obs.satpos[t].XYZPos.z << ' ';
		ft << "Clk=" << std::setw(13) << std::scientific << std::setprecision(6) << obs.satpos[t].clock << ' ';
		ft << "Vx=" << std::setw(13) << std::fixed << std::setprecision(4) << obs.satpos[t].XYZVel.x << ' ';
		ft << "Vy=" << std::setw(13) << obs.satpos[t].XYZVel.y << ' ';
		ft << "Vz=" << std::setw(13) << obs.satpos[t].XYZVel.z << ' ';
		ft << "Clkd=" << std::setw(13) << std::scientific << std::setprecision(5) << obs.satpos[t].clockVelocity << ' ';
		ft << "PIF=" << std::setw(13) << std::fixed << std::setprecision(4) << obs.satpos[t].PIF << ' ';
		ft << "Trop=" << std::setw(13) << std::setprecision(3) << obs.satpos[t].Trop << ' ';
		ft << "El=" << std::setw(13) << obs.satpos[t].El << "deg" << ' ';
		//ft << "Status=" << std::setw(13) << obs.satpos[t].Status << ' ';
		ft << '\n';
	}
}
/// <summary>
/// 误差分析输出
/// </summary>
/// <param name="ft"></param>
/// <param name="obs"></param>
/// <param name="postion"></param>
/// <param name="base"></param>
void _ErrorAnalysis(std::ofstream& ft, rover& rover, BASESTATION& base)
{
	if (fabs(rover._BLHpostion.h) < EPS)
		return;
	BLH2XYZ(base.BLHBESTPOS, base.XYZBESTPOS, a_GPS, alpha_GPS);
	ENU e;
	XYZ2ENU(rover._XYZpostion, base.BLHBESTPOS, e, a_GPS, alpha_GPS);
	ft << rover.gt.Week << ' ' << std::fixed << std::setprecision(3) << rover.gt.SecOfWeek << ' ';
	ft << std::setw(13) << std::fixed << std::setprecision(4) << rover._XYZpostion.x << ' ';
	ft << std::setw(13) << rover._XYZpostion.y << ' ';
	ft << std::setw(13) << rover._XYZpostion.z << ' ';
	ft << std::setw(13) << std::setprecision(8) << rover._BLHpostion.b << ' ';
	ft << std::setw(13) << rover._BLHpostion.l << ' ';
	ft << std::setw(13) << std::setprecision(3) << rover._BLHpostion.h << ' ';
	ft << std::setw(12) << rover.RcvClkOft[0] << ' ';
	ft << std::setw(12) << rover.RcvClkOft[1] << ' ';
	ft << std::setw(8) << rover.PDOP << ' ';
	ft << std::setw(8) << rover.SigmaPos << ' ';
	ft << std::setw(12) << rover.RcvClkSft << ' ';
	ft << std::setw(8) << rover.VDOP << ' ';
	ft << std::setw(8) << rover.SigmaVel << ' ';
	ft << std::dec << std::setw(3) << rover.GPSSatNum << ' ';
	ft << std::setw(3) << rover.BDSSatNum << ' ';
	ft << std::setw(3) << rover.AllSatNum << ' ';
	ft << std::setw(13) << std::fixed << std::setprecision(4) << rover.Velocity[0] << ' ';
	ft << std::setw(13) << rover.Velocity[1] << ' ';
	ft << std::setw(13) << rover.Velocity[2] << ' ';
	ft << std::setw(13) << std::fixed << std::setprecision(4) << rover._XYZpostion.x - base.XYZBESTPOS.x << ' ';
	ft << std::setw(13) << rover._XYZpostion.y - base.XYZBESTPOS.y << ' ';
	ft << std::setw(13) << rover._XYZpostion.z - base.XYZBESTPOS.z << ' ';
	ft << std::setw(13) << std::setprecision(8) << rover._BLHpostion.b - base.BLHBESTPOS.b << ' ';
	ft << std::setw(13) << rover._BLHpostion.l - base.BLHBESTPOS.l << ' ';
	ft << std::setw(13) << std::setprecision(3) << rover._BLHpostion.h - base.BLHBESTPOS.h << ' ';
	ft << std::setw(13) << std::fixed << std::setprecision(4) << e.e  << ' ';
	ft << std::setw(13) << e.n << ' ';
	ft << std::setw(13) << e.u << ' ';
	ft << std::setw(8) << rover.HDOP << ' ';
	ft << std::setw(8) << rover.VDOP << ' ';
	ft << std::setw(8) << rover.PDOP * rover.SigmaPos << ' ';
	ft << '\n';
}
/// <summary>
/// 结果文件输出
/// </summary>
/// <param name="ft"></param>
/// <param name="obs"></param>
/// <param name="postion"></param>
void _ResultFile(std::ofstream& ft, OBS& obs, rover&rover)
{
	_OUTSatPostionFile(ft, obs);
	_OUTPostionFile(ft, rover);
}

/// <summary>
/// 控制台输出
/// </summary>
/// <param name="ft"></param>
/// <param name="obs"></param>
/// <param name="posresult"></param>
void _ConsoleResult(rover& rover)
{
	std::cout << toStr(rover)<<':'<<' ' << rover.gt.Week << ' ' << std::fixed << std::setprecision(3) << rover.gt.SecOfWeek << ' ';
	std::cout << "X:" << std::setw(13) << std::fixed << std::setprecision(4) << rover._XYZpostion.x << ' ';
	std::cout << "Y:" << std::setw(13) << rover._XYZpostion.y << ' ';
	std::cout << "Z:" << std::setw(13) << rover._XYZpostion.z << ' ';
	std::cout << "VX:" << std::setw(13) << std::fixed << std::setprecision(4) << rover.Velocity[0] << ' ';
	std::cout << "VY:" << std::setw(13) << rover.Velocity[1] << ' ';
	std::cout << "VZ:" << std::setw(13) << rover.Velocity[2] << ' ';
	std::cout << "B:" << std::setw(13) << std::setprecision(8) << rover._BLHpostion.b << ' ';
	std::cout << "L:" << std::setw(13) << rover._BLHpostion.l << ' ';
	std::cout << "H:" << std::setw(6) << std::setprecision(3) << rover._BLHpostion.h << ' ';
	std::cout << "GPS Clk:" << std::setw(6) << rover.RcvClkOft[0] << ' ';
	std::cout << "BDS Clk:" << std::setw(6) << rover.RcvClkOft[1] << ' ';
	std::cout << "PDOP:" << std::setw(8) << rover.PDOP << ' ';
	std::cout << "SigmaPos:" << std::setw(8) << rover.SigmaPos << ' ';
	std::cout << "GPS:" << std::dec << std::setw(3) << rover.GPSSatNum << ' ';
	std::cout << "BDS:" << std::setw(3) << rover.BDSSatNum << ' ';
	std::cout << "Sats:" << std::setw(3) << rover.AllSatNum << '\n';
}

/// <summary>
/// RTK结果控制台输出
/// </summary>
/// <param name="rover"></param>
/// <param name="rtk"></param>
void _OutRtkResultToConsole(RTKData& rtk)
{
	std::cout << "RTK" << ':' << ' ' << rtk.Roverrover.gt.Week << ' ' << std::fixed << std::setprecision(3) << rtk.Roverrover.gt.SecOfWeek << ' ';
	std::cout << "B:" << std::setw(13) << std::setprecision(8) << rtk.Roverrover._BLHpostion.b << ' ';
	std::cout << "L:" << std::setw(13) << rtk.Roverrover._BLHpostion.l << ' ';
	std::cout << "H:" << std::setw(6) << std::setprecision(3) << rtk.Roverrover._BLHpostion.h << ' ';
	std::cout << "dX:" << std::setw(13) << std::fixed << std::setprecision(8) << rtk.DDObs.dPos[0] << ' ';
	std::cout << "dY:" << std::setw(13) << rtk.DDObs.dPos[1] << ' ';
	std::cout << "dZ:" << std::setw(13) << rtk.DDObs.dPos[2] << ' ';
	std::cout << "SigmaPos:" << std::setw(13) << rtk.Roverrover.SigmaPos << ' ';
	std::cout << "ResAmb[0]:" << std::setw(13) << std::setprecision(8) << rtk.DDObs.ResAmb[0] << ' ';
	std::cout << "ResAmb[1]:" << std::setw(13) << std::setprecision(8) << rtk.DDObs.ResAmb[1] << ' ';
	std::cout << "Ratio:" << std::setw(12) << std::setprecision(5) << rtk.DDObs.Ratio << ' ';
	std::cout << "RefPRN: G" << std::dec << std::setw(2) << rtk.DDObs.RefPrn[0] << ' ';
	std::cout << "C" << std::dec << std::setw(2) << rtk.DDObs.RefPrn[1] << ' ';
	std::cout << "GPS:" << std::dec << std::setw(3) << rtk.DDObs.DDSatNum[0] << ' ';
	std::cout << "BDS:" << std::setw(3) << rtk.DDObs.DDSatNum[0] << ' ';
	std::cout << "Sats:" << std::setw(3) << rtk.Roverrover.AllSatNum << ' ';
	std::cout << "IsFixed:" << std::setw(3) << rtk.DDObs.bFixed << '\n';

}

/// <summary>
/// 输出RTK定位结果
/// </summary>
/// <param name="rtk"></param>
void _OutRtkResultToFile(std::ofstream& ft, RTKData& rtk)
{
	ft << "RTK:" << rtk.Baserover.gt.Week << " B:" << std::fixed << std::setprecision(3) << rtk.Baserover.gt.SecOfWeek << ' ';
	ft << "R:" << std::fixed << std::setprecision(3) << rtk.Roverrover.gt.SecOfWeek << ' ';
	ft << "B:" << std::setw(13) << std::setprecision(8) << rtk.Roverrover._BLHpostion.b << ' ';
	ft << "L:" << std::setw(13) << rtk.Roverrover._BLHpostion.l << ' ';
	ft << "H:" << std::setw(8) << std::setprecision(3) << rtk.Roverrover._BLHpostion.h << ' ';
	ft << "Bx:" << std::setw(13) << std::fixed << std::setprecision(4) << rtk.basesta.XYZBESTPOS.x << ' ';
	ft << "By:" << std::setw(13) << rtk.basesta.XYZBESTPOS.y << ' ';
	ft << "Bz:" << std::setw(13) << rtk.basesta.XYZBESTPOS.z << ' ';
	ft << "Rx:" << std::setw(13) << std::fixed << std::setprecision(4) << rtk.Roverrover._XYZpostion.x << ' ';
	ft << "Ry:" << std::setw(13) << rtk.Roverrover._XYZpostion.y << ' ';
	ft << "Rz:" << std::setw(13) << rtk.Roverrover._XYZpostion.z << ' ';
	ft << "dX:" << std::setw(13) << std::fixed << std::setprecision(8) << rtk.DDObs.dPos[0] << ' ';
	ft << "dY:" << std::setw(13) << rtk.DDObs.dPos[1] << ' ';
	ft << "dZ:" << std::setw(13) << rtk.DDObs.dPos[2] << ' ';
	ft << "SigmaPos:" << std::setw(13) << rtk.Roverrover.SigmaPos << ' ';
	ft << "mxx:" << std::setw(13) << rtk.Roverrover.mxyz[0] << ' ';
	ft << "myy:" << std::setw(13) << rtk.Roverrover.mxyz[1] << ' ';
	ft << "mzz:" << std::setw(13) << rtk.Roverrover.mxyz[2] << ' ';
	ft << "PDOP:" << std::setw(13) << rtk.Roverrover.PDOP << ' ';
	ft << "HDOP:" << std::setw(13) << rtk.Roverrover.HDOP << ' ';
	ft << "VDOP:" << std::setw(13) << rtk.Roverrover.VDOP << ' ';
	ft << "ResAmb[0]:" << std::setw(13) << std::setprecision(8) << rtk.DDObs.ResAmb[0] << ' ';
	ft << "ResAmb[1]:" << std::setw(13) << std::setprecision(8) << rtk.DDObs.ResAmb[1] << ' ';
	ft << "Ratio:" << std::setw(13) << std::setprecision(8) << rtk.DDObs.Ratio << ' ';
	ft << "RefPRN: G" << std::dec << std::setw(2) << rtk.DDObs.RefPrn[0] << ' ';
	ft << "C" << std::dec << std::setw(2) << rtk.DDObs.RefPrn[1] << ' ';
	ft << "rtk sats: ";
	for (int i = 0; i < rtk.SDobs.SatNum; i++)
	{
		if (!rtk.SDobs.SdSatObs[i].Valid)continue;
		if (rtk.SDobs.SdSatObs[i].PRN == 0)continue;
		if (rtk.SDobs.SdSatObs[i].half == 0)continue;
		if (rtk.SDobs.SdSatObs[i].system == GPS)
			ft << "G" << std::dec << std::setw(2) << rtk.SDobs.SdSatObs[i].PRN << ' ';
		else if (rtk.SDobs.SdSatObs[i].system == BDS)
			ft << "C" << std::dec << std::setw(2) << rtk.SDobs.SdSatObs[i].PRN << ' ';
	}
	ft << "GPS:" << std::dec << std::setw(3) << rtk.DDObs.DDSatNum[0] << ' ';
	ft << "BDS:" << std::setw(3) << rtk.DDObs.DDSatNum[1] << ' ';
	ft << "Sats:" << std::setw(3) << rtk.Roverrover.AllSatNum << ' ';
	ft << "IsFixed:" << std::setw(3) << rtk.DDObs.bFixed << '\n';
}

/// <summary>
/// 输出以可视化
/// </summary>
/// <param name="ft"></param>
/// <param name="rtk"></param>
void _OutRtkErrorToFile(std::ofstream& ft, RTKData& rtk)
{
	ENU e;
	XYZ2ENU(rtk.Roverrover._XYZpostion, rtk.basesta.BLHBESTPOS, e, a_GPS, alpha_GPS);
	ft << rtk.Roverrover.gt.Week << ' ' << std::fixed << std::setprecision(3) << rtk.Roverrover.gt.SecOfWeek << ' ';//<< "RTK:" 
	ft << std::setw(13) << std::setprecision(8) << rtk.Roverrover._BLHpostion.b << ' ';//<< "B:" 
	ft << std::setw(13) << rtk.Roverrover._BLHpostion.l << ' ';//<< "L:" 
	ft << std::setw(8) << std::setprecision(3) << rtk.Roverrover._BLHpostion.h << ' ';//<< "H:" 
	ft << std::setw(13) << std::fixed << std::setprecision(4) << rtk.Roverrover._XYZpostion.x << ' ';//<< "X:" 
	ft << std::setw(13) << rtk.Roverrover._XYZpostion.y << ' '; //<< "Y:"
	ft << std::setw(13) << rtk.Roverrover._XYZpostion.z << ' '; //<< "Z:"
	ft << std::setw(13) << std::fixed << std::setprecision(8) << rtk.DDObs.dPos[0] << ' ';//<< "dX:" 
	ft << std::setw(13) << rtk.DDObs.dPos[1] << ' ';//<< "dY:" 
	ft << std::setw(13) << rtk.DDObs.dPos[2] << ' ';//<< "dZ:" 
	ft << std::setw(13) << rtk.Roverrover.SigmaPos << ' ';//<< "SigmaPos:" 
	ft << std::setw(13) << rtk.Roverrover.PDOP << ' ';//<< "PDOP:" 
	ft << std::setw(13) << rtk.Roverrover.HDOP << ' ';//<< "HDOP:" 
	ft << std::setw(13) << rtk.Roverrover.VDOP << ' ';//<< "VDOP:" 
	ft << std::setw(13) << std::setprecision(8) << rtk.DDObs.Ratio << ' ';//<< "Ratio:" 
	ft << std::setw(13) << std::setprecision(8) << rtk.DDObs.ResAmb[0] << ' ';// << "ResAmb[0]:"
	ft << std::setw(13) << std::setprecision(8) << rtk.DDObs.ResAmb[1] << ' ';//<< "ResAmb[1]:" 
	ft << std::dec << std::setw(3) << rtk.DDObs.DDSatNum[0] << ' ';//<< "GPS:" 
	ft << std::setw(3) << rtk.DDObs.DDSatNum[1] << ' ';//<< "BDS:" 
	ft << std::setw(3) << rtk.Roverrover.AllSatNum << ' ';//<< "Sats:" 
	ft << std::setw(13) << std::fixed << std::setprecision(4) << e.e << ' ';//<< "E:" 
	ft << std::setw(13) << e.n << ' '; //<< "N:"
	ft << std::setw(13) << e.u << ' '; //<< "U:"
	ft << std::setw(3) << rtk.DDObs.bFixed << ' ';//<< "IsFixed:" 
	ft << std::setw(13) << std::setprecision(8) << rtk.Roverrover.mxyz[0] << ' ';
	ft << std::setw(13) << rtk.Roverrover.mxyz[1] << ' ';
	ft << std::setw(13) << rtk.Roverrover.mxyz[2] << '\n';
}