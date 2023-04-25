#include"RTk.h"

/// <summary>
/// kalman ��ʼ��
/// </summary>
/// <param name="rover">����վ����</param>
/// <param name="SDObs">����۲�ֵ</param>
/// <param name="DDCObs">˫��۲�ֵ</param>
/// <param name="kalmanLast">�ϸ���Ԫ��kalman</param>
/// <returns>�Ƿ��Ѿ���ʼ��</returns>
void KalmanInitialize(rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanNow)
{
	//��ȡ�۲�ֵʱ��
	kalmanNow.gt = SDObs.gt;
	//��ȡ״̬��ά��
	kalmanNow.dim = 3 + 2 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	//��ȡ�ο���
	kalmanNow.RefPrn[0] = DDCObs.RefPrn[0];
	kalmanNow.RefPrn[1] = DDCObs.RefPrn[1];
	//��ȡ���Ǳ�ǩ
	int row = 0;//�ܹ�dim-3��
	int satnum = 0;
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1])continue;
		//��ȡ������������prn
		kalmanNow.PRN[satnum] = SDObs.SdSatObs[i].PRN;
		kalmanNow.system[satnum] = SDObs.SdSatObs[i].system;
		satnum++;
		for (int k = 0; k < 2; k++)
		{
			//ϵͳ��ǩ
			if (SDObs.SdSatObs[i].system == GPS)
			{
				kalmanNow.PRNLable[row][0] = GPS;
				kalmanNow.PRNLable[row][1] = DDCObs.RefPrn[0];//�ο���
			}
			else if (SDObs.SdSatObs[i].system == BDS)
			{
				kalmanNow.PRNLable[row][0] = BDS;
				kalmanNow.PRNLable[row][1] = DDCObs.RefPrn[1];//�ο���
			}
			kalmanNow.PRNLable[row][2] = SDObs.SdSatObs[i].PRN;
			kalmanNow.PRNLable[row][3] = k + 1;//Ƶ�ʱ�ǩ

			row++;
		}

	}
	//״̬����ʼ��
	kalmanNow.X[0] = rover._XYZpostion.x; kalmanNow.X[1] = rover._XYZpostion.y; kalmanNow.X[2] = rover._XYZpostion.z;
	int Ncol = 3;
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1])continue;
		for (int k = 0; k < 2; k++)
		{
			if (SDObs.SdSatObs[i].system == GPS)
			{
				kalmanNow.X[Ncol] = ((SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k])
					- (SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dP[k]))
					 / (Cspeed / SDObs.SdSatObs[i].f[k]);
				Ncol++;
			}
			else if (SDObs.SdSatObs[i].system == BDS)
			{
				kalmanNow.X[Ncol] = ((SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k])
					- (SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dP[k]))
					/ (Cspeed / SDObs.SdSatObs[i].f[k]);
				Ncol++;
			}
			else continue;
		}
	}
	//Э����
	for (int i = 0; i < kalmanNow.dim; i++)
	{
		if (i < 3) 
			kalmanNow.XCov[i * kalmanNow.dim + i] = pow(5, 2); 
		else
			kalmanNow.XCov[i * kalmanNow.dim + i] = pow(5, 2);//��^2
	}

}

/// <summary>
/// ����Phi
/// </summary>
/// <param name="kalmanLast"></param>
/// <param name="kalmanNow"></param>
/// <returns>Phi����</returns>
Matrix _GetPhi(Kalman& kalmanLast, Kalman& kalmanNow)
{
	int rows = kalmanNow.dim;
	int cols = kalmanLast.dim;
	double* phi = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		phi[i] = 0.0;

	//˫��ѭ��
	for (int i = 0; i < 3; i++)
		phi[i * cols + i] = 1.0;
	for (int i = 3; i < rows; i++)
	{
		for (int j = 3; j < cols; j++)
		{
			if (kalmanNow.PRNLable[i - 3][3] != kalmanLast.PRNLable[j - 3][3])continue;//Ƶ��
			if (kalmanNow.PRNLable[i - 3][0] != kalmanLast.PRNLable[j - 3][0])continue;//����ϵͳ
			if (kalmanNow.PRNLable[i - 3][1] == kalmanLast.PRNLable[j - 3][1])//�ο�����ͬ
			{
				if (kalmanNow.PRNLable[i - 3][2] != kalmanLast.PRNLable[j - 3][2])continue;//�����������ͬ
				phi[i * cols + j] = 1.0;
				break;
			}
			else
			{
				if (kalmanNow.PRNLable[i - 3][2] == kalmanLast.PRNLable[j - 3][2])//�������Ǻ���ͬϵ������1
					phi[i * cols + j] = 1.0;
				if (kalmanNow.PRNLable[i - 3][1] == kalmanLast.PRNLable[j - 3][2])//��ʱ�̵Ĳο��ǵ�����һʱ�̵Ľ����ǣ�ϵ��Ϊ-1
					phi[i * cols + j] = -1.0;

				continue;
			}

		}
	}
	Matrix Phi(rows, cols);
	Phi = phi;
	delete[] phi;
	return Phi;
}

//
Matrix _GetQ(Kalman& kalmanLast, Kalman& kalmanNow)
{
	int rows = kalmanNow.dim;
	int cols = kalmanLast.dim;
	double* Q = new double[rows * rows];
	for (int i = 0; i < rows * rows; i++)
		Q[i] = 0.0;
	//˫��ѭ��
	for (int i = 0; i < 3; i++)
		Q[i * rows + i] = pow(5, 2);
	//ģ����
	int i = 3;
	int j = 3;
	for (; i < rows; i++)
	{
		j = 3;
		for (; j < cols; j++)
		{
			if (kalmanNow.PRNLable[i - 3][3] != kalmanLast.PRNLable[j - 3][3])continue;//Ƶ��
			if (kalmanNow.PRNLable[i - 3][0] != kalmanLast.PRNLable[j - 3][0])continue;//����ϵͳ
			if (kalmanNow.PRNLable[i - 3][2] != kalmanLast.PRNLable[j - 3][2])continue;//�����������ͬ
			Q[i * rows + i] = EPS;
			break;
		}
		if(kalmanLast.dim==0||j==cols)
			Q[i * rows + i] = pow(5, 2);

	}


	Matrix mQ(rows, rows);
	mQ = Q;

	delete[] Q;
	return mQ;

}


Matrix _GetH(OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1)
{
	int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int cols = Xk_k1.row();
	//1.���û�վ������վ��ֵ
	XYZ roverPostion;//��ʱ��Ԥ���״̬������г�ʼ��
	roverPostion.x = Xk_k1(0, 0);
	roverPostion.y = Xk_k1(1, 0);
	roverPostion.z = Xk_k1(2, 0);

	//2.����GPS��BDS��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];

	double* arrayH = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayH[i] = 0.0;
	int row = 0;
	int col = 0;

	double l = 0.0;
	double m = 0.0;
	double n = 0.0;
	//����վ����֮��ľ���
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		PR[i] = distance(roverPostion, roverObs.satpos, SDObs.SdSatObs[i].nRov);
	}

	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
		if (SDObs.SdSatObs[i].system == GPS)
		{
			if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
				- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.x) / PR[DDCObs.RefPos[0]];
			m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
				- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.y) / PR[DDCObs.RefPos[0]];
			n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
				- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.z) / PR[DDCObs.RefPos[0]];
		}
		else if (SDObs.SdSatObs[i].system == BDS)
		{
			if (!DDCObs.RefStatus[1]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
				- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.x) / PR[DDCObs.RefPos[1]];
			m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
				- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.y) / PR[DDCObs.RefPos[1]];
			n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
				- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.z) / PR[DDCObs.RefPos[1]];
		}
		else continue;
		for (int k = 0; k < 4; k++)
		{
			arrayH[cols * row] = l; arrayH[cols * row + 1] = m; arrayH[cols * row + 2] = n;
			if (k > 1)
			{
				arrayH[cols * row + 3 + col] = Cspeed / SDObs.SdSatObs[i].f[k - 2];
				col++;
			}
			row++;
		}

	}
	Matrix H(rows, cols);
	H = arrayH;

	delete[] arrayH;
	delete[] PR;

	return H;
}

Matrix _GetHFixed(OBS& roverObs, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1)
{
	int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int cols = Xk_k1.row();
	//1.���û�վ������վ��ֵ
	XYZ roverPostion;//��ʱ��Ԥ���״̬������г�ʼ��
	roverPostion.x = Xk_k1(0, 0);
	roverPostion.y = Xk_k1(1, 0);
	roverPostion.z = Xk_k1(2, 0);

	//2.����GPS��BDS��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];

	double* arrayH = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayH[i] = 0.0;
	int row = 0;
	int col = 0;

	double l = 0.0;
	double m = 0.0;
	double n = 0.0;
	//����վ����֮��ľ���
	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		PR[i] = distance(roverPostion, roverObs.satpos, SDObs.SdSatObs[i].nRov);
	}

	for (int i = 0; i < SDObs.SatNum; i++)
	{
		if (SDObs.SdSatObs[i].Valid != 1)continue;
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
		if (SDObs.SdSatObs[i].system == GPS)
		{
			if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
				- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.x) / PR[DDCObs.RefPos[0]];
			m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
				- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.y) / PR[DDCObs.RefPos[0]];
			n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
				- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[0]].nRov].XYZPos.z) / PR[DDCObs.RefPos[0]];
		}
		else if (SDObs.SdSatObs[i].system == BDS)
		{
			if (!DDCObs.RefStatus[1]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			l = (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.x) / PR[i]
				- (roverPostion.x - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.x) / PR[DDCObs.RefPos[1]];
			m = (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.y) / PR[i]
				- (roverPostion.y - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.y) / PR[DDCObs.RefPos[1]];
			n = (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[i].nRov].XYZPos.z) / PR[i]
				- (roverPostion.z - roverObs.satpos[SDObs.SdSatObs[DDCObs.RefPos[1]].nRov].XYZPos.z) / PR[DDCObs.RefPos[1]];
		}
		else continue;
		for (int k = 0; k < 2; k++)
		{
			arrayH[cols * row] = l; arrayH[cols * row + 1] = m; arrayH[cols * row + 2] = n;
			arrayH[cols * row + 3 + col] = Cspeed / SDObs.SdSatObs[i].f[k];
			row++;
			col++;
		}
		for (int k = 0; k < 2; k++)
		{
			arrayH[cols * row] = 0.0; arrayH[cols * row + 1] = 0.0; arrayH[cols * row + 2] = 0.0;
			arrayH[cols * row + 3 + col-(2-k)] = 1.0;
			row++;
		}
	}
	Matrix H(rows, cols);
	H = arrayH;

	delete[] arrayH;
	delete[] PR;

	return H;

}

Matrix _GetZ(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs,  Kalman& kalmanNow)
{
	int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int cols = 1;
	double* arrayZ = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayZ[i] = 0.0;
	//1.���û�վ������վ��ֵ
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion;//��ʱ��Ԥ���״̬������г�ʼ��
	roverPostion.x = kalmanNow.X[0];
	roverPostion.y = kalmanNow.X[1];
	roverPostion.z = kalmanNow.X[2];
	//2.����GPS��BDS��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];
	double* N = new double[kalmanNow.dim-3];//ģ���ȳ�ֵ //����и�ֵ //��λ��α��
	for (int i = 0; i < kalmanNow.dim-3; i++)
		N[i] = kalmanNow.X[i + 3];
	int row = 0;
	int col = 0;
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
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
		if (SDObs.SdSatObs[i].system == GPS)
		{
			if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
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
				arrayZ[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dP[k]
					- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
					+ l * roverPostion.x + m * roverPostion.y + n * roverPostion.z;
				row++;
			}
			//����������
			for (int k = 0; k < 2; k++)
			{
				arrayZ[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
					- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
					+ l * roverPostion.x + m * roverPostion.y + n * roverPostion.z;
					//- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
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
				arrayZ[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dP[k]
					- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
					+ l * roverPostion.x + m * roverPostion.y + n * roverPostion.z;
				row++;
			}
			//��������
			for (int k = 0; k < 2; k++)
			{
				arrayZ[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
					- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
					+ l * roverPostion.x + m * roverPostion.y + n * roverPostion.z;
					//- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
				row++; col++;
			}

		}

	}

	Matrix Z(rows, cols);
	Z = arrayZ;


	delete[] arrayZ;
	delete[] PR;
	delete[] PB;

	return Z;

}

Matrix _GetR(int GPSSatNum,int BDSSatNum)
{
	int rows = 4 * (GPSSatNum+BDSSatNum);
	int cols = rows;
	double* arrayR = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayR[i] = 0.0;

	double sigmaP = pow(2,2);
	double sigmaL = pow(0.02,2);

	for (int i = 0; i < 4 * GPSSatNum; i++)
	{
		if (i % 4 <  2)
			arrayR[i * rows + i] = 4 * sigmaP;
		else
			arrayR[i * rows + i] = 4 * sigmaL;//���Խ��ߣ�������

		for (int j = 0; j < 4 * GPSSatNum; j = j + 1)//Э����
		{
			if (i == j)continue;
			if (abs(i-j) % 4 > 0)continue;
			if (i % 4 < 2)
				arrayR[i * rows + j] = 2 * sigmaP;
			else
				arrayR[rows * i + j] = 2 * sigmaL;
		}

	}

	for (int i = 4 * GPSSatNum; i < rows; i++)
	{
		if (i % 4 < 2)
			arrayR[i * rows + i] = 4 * sigmaP;
		else
			arrayR[i * rows + i] = 4 * sigmaL;//���Խ��ߣ�������
		for (int j = 4 * GPSSatNum; j < rows; j++)//Э����
		{
			if (i==j)continue;
			if (abs(i-j) % 4 > 0)continue;
			if (i % 4 < 2)
				arrayR[i * rows + j] = 2 * sigmaP;
			else
				arrayR[rows * i + j] = 2 * sigmaL;
		}
	}

	Matrix R(rows, cols);
	R = arrayR;
	delete[] arrayR;

	return R;

}

Matrix _GetRFixed(int GPSSatNum, int BDSSatNum)
{
	int rows = 4 * (GPSSatNum + BDSSatNum);
	int cols = rows;
	double* arrayR = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayR[i] = 0.0;

	//double sigmaP = pow(2, 2);
	double sigmaL = pow(0.02, 2);

	for (int i = 0; i < 4 * GPSSatNum; i++)
	{
		if (i % 4 < 2)
			arrayR[i * rows + i] = 4 * sigmaL;
		else
			arrayR[i * rows + i] = EPS;//���Խ��ߣ�������

		for (int j = 0; j < 4 * GPSSatNum; j = j + 1)//Э����
		{
			if (i == j)continue;
			if (abs(i - j) % 4 > 0)continue;
			if (i % 4 < 2)
				arrayR[i * rows + j] = 2 * sigmaL;
			else
				continue;
		}

	}

	for (int i = 4 * GPSSatNum; i < rows; i++)
	{
		if (i % 4 < 2)
			arrayR[i * rows + i] = 4 * sigmaL;
		else
			arrayR[i * rows + i] = 4 * EPS;//���Խ��ߣ�������
		for (int j = 4 * GPSSatNum; j < rows; j++)//Э����
		{
			if (i == j)continue;
			if (abs(i - j) % 4 > 0)continue;
			if (i % 4 < 2)
				arrayR[i * rows + j] = 2 * sigmaL;
			else
				continue;
		}
	}

	Matrix R(rows, cols);
	R = arrayR;
	delete[] arrayR;

	return R;

}

Matrix _GetV(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1)
{
	int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int cols = 1;
	double* arrayV = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayV[i] = 0.0;
	//1.���û�վ������վ��ֵ
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion;//��ʱ��Ԥ���״̬������г�ʼ��
	roverPostion.x = Xk_k1(0, 0);
	roverPostion.y = Xk_k1(1, 0);
	roverPostion.z = Xk_k1(2, 0);
	//2.����GPS��BDS��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];
	double* N = new double[Xk_k1.row() - 3];//ģ���ȳ�ֵ //����и�ֵ //��λ��α��
	for (int i = 0; i < Xk_k1.row() - 3; i++)
		N[i] = Xk_k1(i + 3, 0);
	int row = 0;
	int col = 0;
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
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
		if (SDObs.SdSatObs[i].system == GPS)
		{
			if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			///ÿ����һ��ѭ��
			//��һ������
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dP[k]
					- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]]);
				row++;
			}
			//����������
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
					- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
					- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
				row++; col++;
			}

		}
		else if (SDObs.SdSatObs[i].system == BDS)
		{
			if (!DDCObs.RefStatus[1]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			///ÿ����һ��ѭ��
			//��һ����
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = SDObs.SdSatObs[i].dP[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dP[k]
					- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]]);
				row++;
			}
			//��������
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
					- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
					- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
				row++; col++;
			}

		}

	}

	Matrix V(rows, cols);
	V = arrayV;


	delete[] arrayV;
	delete[] PR;
	delete[] PB;
	delete[] N;


	return V;
}

Matrix _GetVFixed(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Matrix& Xk_k1)
{
	int rows = 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]);
	int cols = 1;
	double* arrayV = new double[rows * cols];
	for (int i = 0; i < rows * cols; i++)
		arrayV[i] = 0.0;
	//1.���û�վ������վ��ֵ
	XYZ basePostion = base.XYZBESTPOS;
	XYZ roverPostion;//��ʱ��Ԥ���״̬������г�ʼ��
	roverPostion.x = Xk_k1(0, 0);
	roverPostion.y = Xk_k1(1, 0);
	roverPostion.z = Xk_k1(2, 0);
	//2.����GPS��BDS��˫��������
	//3/4����վ���굽�������ǵļ��ξ���
	//������۲�ֵ����������棬û��ͨ���ֲ�̽���û�о���
	double* PR = new double[SDObs.SatNum];
	double* PB = new double[SDObs.SatNum];
	double* N = new double[Xk_k1.row() - 3];//ģ���ȳ�ֵ //����и�ֵ //��λ��α��
	double* NFixed = new double[Xk_k1.row() - 3];//ģ���ȳ�ֵ //����и�ֵ //��λ��α��
	for (int i = 0; i < Xk_k1.row() - 3; i++)
		N[i] = Xk_k1(i + 3, 0);
	for (int i = 0; i < Xk_k1.row() - 3; i++)
		NFixed[i] = DDCObs.FixedAmb[i];
	int row = 0;
	int col = 0;
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
		if (SDObs.SdSatObs[i].half != 1)continue;//�а��ܵĲ�����RTk
		if (i == DDCObs.RefPos[0] || i == DDCObs.RefPos[1]) continue;//��׼�ǲ�����
		if (SDObs.SdSatObs[i].system == GPS)
		{
			if (!DDCObs.RefStatus[0]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			///ÿ����һ��ѭ��
			//��һ������
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[0]].dL[k]
					- (PR[i] - PR[DDCObs.RefPos[0]] - PB[i] + PB[DDCObs.RefPos[0]])
					- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
				row++; col++;

			}
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = NFixed[col-(2-k)] - N[col-(2-k)];
				row++;
			}
		}
		else if (SDObs.SdSatObs[i].system == BDS)
		{
			if (!DDCObs.RefStatus[1]) continue;//û��ѡ����׼�ǵĴ�����ǵ���۲�ֵû��ͨ���ֲ����
			///ÿ����һ��ѭ��
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = SDObs.SdSatObs[i].dL[k] - SDObs.SdSatObs[DDCObs.RefPos[1]].dL[k]
					- (PR[i] - PR[DDCObs.RefPos[1]] - PB[i] + PB[DDCObs.RefPos[1]])
					- Cspeed / SDObs.SdSatObs[i].f[k] * N[col];
				row++; col++;

			}
			for (int k = 0; k < 2; k++)
			{
				arrayV[row] = NFixed[col - (2 - k)] - N[col - (2 - k)];
				row++;
			}

		}

	}

	Matrix V(rows, cols);
	V = arrayV;


	delete[] arrayV;
	delete[] PR;
	delete[] PB;
	delete[] N;
	delete[] NFixed;

	return V;

}
/// <summary>
/// 
/// </summary>
/// <param name="kalmanLast"></param>
/// <param name="kalmanNow"></param>
/// <param name="Pk_k1"></param>
/// <param name="Xk_k1"></param>
void EKFPredict( Kalman& kalmanLast, Kalman& kalmanNow,Matrix& Pk_k1,Matrix& Xk_k1)
{
	//ʱ��Ԥ��
	// ����
	Matrix Xk_1(kalmanLast.dim, 1);
	Matrix Phi(kalmanNow.dim, kalmanLast.dim);
	Matrix Pk_1(kalmanLast.dim, kalmanLast.dim);
	Matrix Q(kalmanNow.dim, kalmanNow.dim);

	//Xk_1
	Xk_1 = kalmanLast.X;
	//Xk_1.Show();

	//Phi 
	Phi = _GetPhi(kalmanLast, kalmanNow);
	//Phi.Show();

	// Pk_1
	Pk_1 = kalmanLast.XCov;
	//Pk_1.Show();
	
	// Q
	Q = _GetQ(kalmanLast, kalmanNow);
	//Q.Show();
	
	// Pk_k1
	Pk_k1 = Phi * Pk_1 * Phi.Transpose() + Q;
	//Pk_k1.Show();

	//Xk_k1
	Xk_k1 = Phi * Xk_1;
	//Xk_k1.Show();
	//�������������������������³�ʼ��
	for (int i = 0; i < Xk_k1.row(); i++)
	{
		if (fabs(Xk_k1(i, 0)) < EPS)
			Xk_k1.set(i, 0, kalmanNow.X[i]);
		else
			continue;
	}

	//Xk_k1.Show();

}

void EKFUpdate(OBS& baseObs, OBS& roverObs, BASESTATION& base, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, 
	Kalman& kalmanNow, Matrix& Pk_k1, Matrix& Xk_k1,Matrix& Pk,Matrix& Xk)
{
	//�������
	Matrix H(4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]), kalmanNow.dim);
	Matrix Z(4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]), 1);
	Matrix R(4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]), 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]));
	
	Matrix K(kalmanNow.dim, 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]));
	Matrix V(kalmanNow.dim, 1);
	Matrix I(kalmanNow.dim, 1.0);

	// 1.H
	H = _GetH(roverObs, SDObs, DDCObs, Xk_k1);
	//H.Show();
	V = _GetV(baseObs, roverObs, base, SDObs, DDCObs, Xk_k1);
	//V.Show();
	// 3.R
	R = _GetR(DDCObs.DDSatNum[0], DDCObs.DDSatNum[1]);
	//R.Show();
	// 4.K
	K = Pk_k1 * H.Transpose() * (H * Pk_k1 * H.Transpose() + R).Inverse();
	//K.Show();
	// 5.Xk
	//Xk = Xk_k1 + K * (Z - H * Xk_k1);
	Xk = Xk_k1 + K * V;
	//Xk.Show();
	// 6.Pk
	Pk = (I - K * H) * Pk_k1 * (I - K * H).Transpose() + K * R * K.Transpose();
	//Pk.Show();

	//for (int i = 0; i < kalmanNow.dim; i++)
	//{
	//	kalmanNow.X[i] = Xk(i, 0);
	//	for (int j = 0; j < kalmanNow.dim; j++)
	//	{
	//		kalmanNow.XCov[i * kalmanNow.dim + j] = Pk(i, j);
	//	}
	//}

	////for (int i = 3; i < kalmanNow.dim; i++)
	////{
	////	DDCObs.FloatAmb[i - 3] = Xk(i, 0);
	////	for (int j = 3; j < kalmanNow.dim; j++)
	////		DDCObs.FloatAmbCov[(i - 3) * (kalmanNow.dim - 3) + (j - 3)] = Pk(i, j);
	////}

	//Matrix P(4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]), 4 * (DDCObs.DDSatNum[0] + DDCObs.DDSatNum[1]));
	//Matrix Q(kalmanNow.dim, kalmanNow.dim);
	//P = R.Inverse();
	//Q = (H.Transpose() * P * H).Inverse();
	//for (int i = 3; i < kalmanNow.dim; i++)
	//{
	//	DDCObs.FloatAmb[i - 3] = Xk(i, 0);
	//	for (int j = 3; j < kalmanNow.dim; j++)
	//		DDCObs.FloatAmbCov[(i - 3) * (kalmanNow.dim - 3) + (j - 3)] = Q(i, j);
	//}

}
//
bool RTKFloatKalman(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanLast, Kalman& kalmanNow)
{
	//��ʼ��
	KalmanInitialize(rover, SDObs, DDCObs, kalmanNow);
	//�Ƿ��ʼ��Ԫ
	double dt = (kalmanNow.gt.Week - kalmanLast.gt.Week) * 604800 + kalmanNow.gt.SecOfWeek - kalmanLast.gt.SecOfWeek;
	if (dt-1.00>EPS)
	{
		kalmanLast = kalmanNow;
		kalmanNow = Kalman();
		return false;
	}
	Matrix Pk_k1(kalmanNow.dim, kalmanNow.dim);
	Matrix Xk_k1(kalmanNow.dim, 1);
	EKFPredict(kalmanLast, kalmanNow, Pk_k1, Xk_k1);
	//Pk_k1.Show();
	//Xk_k1.Show();

	Matrix Pk(kalmanNow.dim, kalmanNow.dim);
	Matrix Xk(kalmanNow.dim, 1);
	EKFUpdate(baseObs, roverObs, base, SDObs, DDCObs, kalmanNow, Pk_k1, Xk_k1, Pk, Xk);
	//Pk.Show();
	//Xk.Show();

	for (int i = 0; i < kalmanNow.dim; i++)
	{
		kalmanNow.X[i] = Xk(i, 0);
		for (int j = 0; j < kalmanNow.dim; j++) 
		{
			kalmanNow.XCov[i * kalmanNow.dim + j] = Pk(i, j);
		}
	}

	for (int i = 3; i < kalmanNow.dim; i++)
	{
		DDCObs.FloatAmb[i - 3] = Xk(i, 0);
		for (int j = 3; j < kalmanNow.dim; j++)
			DDCObs.FloatAmbCov[(i - 3) * (kalmanNow.dim - 3) + (j - 3)] = Pk(i, j);
	}
	//
	rover._XYZpostion.x = kalmanNow.X[0];
	rover._XYZpostion.y = kalmanNow.X[1];
	rover._XYZpostion.z = kalmanNow.X[2];

	DDCObs.dPos[0] = rover._XYZpostion.x - base.XYZBESTPOS.x;
	DDCObs.dPos[1] = rover._XYZpostion.y - base.XYZBESTPOS.y;
	DDCObs.dPos[2] = rover._XYZpostion.z - base.XYZBESTPOS.z;
	XYZ2BLH(rover._XYZpostion, rover._BLHpostion, a_GPS, alpha_GPS);

	//
	//��Ԫ��������
	kalmanLast = kalmanNow;
	kalmanNow = Kalman();
	return true;
}


bool RTKFixedKalman(OBS& baseObs, OBS& roverObs, BASESTATION& base, rover& rover, SDEPOCHOBS& SDObs, DDCOBS& DDCObs, Kalman& kalmanLast)
{
	//�ٽ���һ���������
	Matrix Xk_k1(kalmanLast.dim, 1);
	Matrix Pk_k1(kalmanLast.dim, kalmanLast.dim);
	Matrix Xk(kalmanLast.dim, 1);
	Matrix Pk(kalmanLast.dim, kalmanLast.dim);
	Xk_k1 = kalmanLast.X;
	Pk_k1 = kalmanLast.XCov;

	Matrix H((kalmanLast.dim - 3) * 2, kalmanLast.dim);
	Matrix V((kalmanLast.dim - 3) * 2, 1);
	Matrix R((kalmanLast.dim - 3) * 2, (kalmanLast.dim - 3) * 2);
	Matrix K(kalmanLast.dim, kalmanLast.dim - 3);
	Matrix I(kalmanLast.dim, 1.0);

	//H
	H = _GetHFixed(roverObs, SDObs, DDCObs, Xk_k1);
	//H.Show();

	//V
	V = _GetVFixed(baseObs, roverObs, base, SDObs, DDCObs, Xk_k1);
	//V.Show();

	//R
	R = _GetRFixed(DDCObs.DDSatNum[0], DDCObs.DDSatNum[1]);
	//R.Show();

	//�������
	K = Pk_k1 * H.Transpose() * (H * Pk_k1 * H.Transpose() + R).Inverse();
	//K.Show();
	Xk = Xk_k1 + K * V;
	//Xk.Show();
	Pk = (I - K * H) * Pk_k1 * (I - K * H).Transpose() + K * R * K.Transpose();

	//���½�֮��Ϊ��һʱ�̵��˲�ֵ��
	int i = 0;
	int j = 0;
	//for (i = 0; i < kalmanLast.dim; i++)
	//{
	//	kalmanLast.X[i] = Xk(i, 0);
	//	for (j = 0; j < kalmanLast.dim; j++)
	//		kalmanLast.XCov[i * kalmanLast.dim + j] = Pk(i, j);
	//}

	rover._XYZpostion.x = Xk(0, 0);
	rover._XYZpostion.y = Xk(1, 0);
	rover._XYZpostion.z = Xk(2, 0);

	XYZ2BLH(rover._XYZpostion, rover._BLHpostion, a_GPS, alpha_GPS);

	DDCObs.dPos[0] = rover._XYZpostion.x - base.XYZBESTPOS.x;
	DDCObs.dPos[1] = rover._XYZpostion.y - base.XYZBESTPOS.y;
	DDCObs.dPos[2] = rover._XYZpostion.z - base.XYZBESTPOS.z;
	return true;
}