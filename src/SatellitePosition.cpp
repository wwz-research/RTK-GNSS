#include "SPP.h"

/****************************************************************************
  SatellitePosition

  目的：计算某历元观测中所有观测到的卫星的位置

  输入参数：
   某历元的观测值和卫星星历
  输出参数：
   obs中的卫星位置、速度、钟差、钟速的计算结果
****************************************************************************/
void SatellitePosition(GPSEPHREC geph[], GPSEPHREC beph[], EPOCHOBSDATA* obs, double UserPos[3])
{
	//循环该历元观测数据中的每颗卫星，计算卫星位置（如果有的卫星位置解算不出来，怎么处理，还可以继续用该历元的观测值去解算接收机的位置吗？丢掉不能用的卫星）
	int i;
	GPSTIME t_Rv = obs->Time;//接收机钟表面时
	GPSTIME t_Sv;//卫星信号发射时刻

	for (i = 0; i < obs->SatNum; i++)
	{

		//计算卫星钟差，同时检查该颗卫星的星历是否过期，星历是否健康
		t_Sv.Week = t_Rv.Week;
		t_Sv.SecOfWeek = t_Rv.SecOfWeek - obs->SatObs[i].P[0] / C_Light;
		if (!CompSatClkOff(obs->SatObs[i].Prn, obs->SatObs[i].System, &t_Sv, geph, beph, &obs->SatPVT[i]))
		{
			obs->SatPVT[i].Valid = false;
			continue;
		}//没有对其返回值进行处理呀，返回false就continue?
		//利用卫星钟差更新两轮信号发射系统时
		t_Sv.SecOfWeek = t_Rv.SecOfWeek - obs->SatObs[i].P[0] / C_Light - obs->SatPVT[i].SatClkOft;
		CompSatClkOff(obs->SatObs[i].Prn, obs->SatObs[i].System, &t_Sv, geph, beph, &obs->SatPVT[i]);
		t_Sv.SecOfWeek = t_Rv.SecOfWeek - obs->SatObs[i].P[0] / C_Light - obs->SatPVT[i].SatClkOft;


		//先判断是GPS还是BDS，再计算卫星位置、速度、钟差、钟速（含有相对论改正），再计算地球自转改正
		if (obs->SatObs[i].System == GPS)
		{
			CompGPSSatPVT(obs->SatObs[i].Prn, &t_Sv, &geph[obs->SatObs[i].Prn - 1], &obs->SatPVT[i]);
			RotationCorrection(&obs->SatPVT[i], UserPos, Omega_WGS);
		}
		else if (obs->SatObs[i].System == BDS)
		{
			CompBDSSatPVT(obs->SatObs[i].Prn, &t_Sv, &beph[obs->SatObs[i].Prn - 1], &obs->SatPVT[i]);
			RotationCorrection(&obs->SatPVT[i], UserPos, Omega_BDS);
		}
		else continue;//代码中多次对这个条件进行容错处理，这一行是否冗余？
		//计算含有相对论的钟差改正后需要重新计算卫星信号发射的系统时吗？不需要，影响很小

	}

}

/****************************************************************************
  CompSatClkOff

  目的：计算卫星钟差

  输入参数：
   Prn号，Sys卫星导航系统，卫星信号发射时刻，卫星星历数据，输入的是卫星中表面时吗？是
  输出参数：
   不加相对论改正的卫星钟差
****************************************************************************/
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid)
{
	double dt, LimT = 7500.0;
	GPSTIME CurT;
	GPSEPHREC* EPH;//利用指针

	CurT = *t;
	if (Sys == GPS) EPH = GPSEph + Prn - 1;
	else if (Sys == BDS) {
		EPH = BDSEph + Prn - 1;
		CurT.Week -= 1356;
		CurT.SecOfWeek -= 14;
		LimT = 3900.0;
	}//注意BDS星历的参考时间为BDT，GPST-BDT=14s
	else return false;

	//星历过期判断，星历健康标记Health是否为0，如果过期或者不健康，返回false，否则返回true
	dt = (CurT.Week - EPH->TOC.Week) * 604800.0 + CurT.SecOfWeek - EPH->TOC.SecOfWeek;
	if (fabs(dt) > LimT || EPH->SVHealth != 0)
	{
		return false;
	}
	Mid->SatClkOft = EPH->ClkBias + EPH->ClkDrift * dt + EPH->ClkDriftRate * pow(dt, 2);//计算卫星钟差
	Mid->Valid = true;//计算成功，Mid->Valid = true; 否则赋值为false
	return true;
}

/****************************************************************************
  CompGPSSatPVT//在WGS84下解算

  目的：计算GPS卫星位置、速度、钟差、钟速（含有相对论改正,但未进行地球自传改正）

  输入参数：
   Prn号，卫星信号发射时刻，单颗卫星星历数据，
  输出参数：
   GPS卫星位置、速度、钟差、钟速（含有相对论改正）
****************************************************************************/
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
	/**********************************计算GPS卫星位置**********************************/
	//1和2. 计算轨道长半轴，计算平均运动角速度
	double n0 = sqrt(GM_Earth) / pow(Eph->SqrtA, 3);
	//3.计算相对于星历参考历元的时间,注意不要改TOE
	double dt = (t->Week - Eph->TOE.Week) * 604800 + (t->SecOfWeek - Eph->TOE.SecOfWeek );
	//4. 对平均运动角速度进行改正
	double n = n0 + Eph->DeltaN;
	//5. 计算平近点角
	double Mk = Eph->M0 + n * dt;
	//6. 计算偏近点角（利用下面的开普勒方程，迭代求解）
	double Ek = Mk;
	double previous_Ek = Mk;
	int max_iterations = 100; // 最大迭代次数
	int iteration_count = 0;  // 迭代计数器
	do {
		previous_Ek = Ek;
		Ek = Mk + Eph->e * sin(Ek);
		iteration_count++; 
		if (iteration_count > max_iterations) { // 如果达到最大迭代次数则强制跳出循环
			break;
		}
	} while (abs(Ek - previous_Ek) > 1e-10); // 迭代精度
	//7. 计算真近点角
	double vk = atan2((sqrt(1 - (Eph->e) * (Eph->e)) * sin(Ek)), (cos(Ek) - Eph->e));
	//8. 计算升交角距
	double upk = vk + Eph->omega;
	//9. 计算二阶调和改正数
	double delta_uk = Eph->Cus * sin(2 * upk) + Eph->Cuc * cos(2 * upk);
	double delta_rk = Eph->Crs * sin(2 * upk) + Eph->Crc * cos(2 * upk);
	double delta_ik = Eph->Cis * sin(2 * upk) + Eph->Cic * cos(2 * upk);
	//10.计算经过改正的升交角距
	double uk = upk + delta_uk;
	//11.计算经过改正的向径
	double rk = pow(Eph->SqrtA, 2) * (1 - (Eph->e) * cos(Ek)) + delta_rk;
	//12.计算经过改正的轨道倾角
	double ik = Eph->i0 + delta_ik + Eph->iDot * dt;
	//13.计算卫星在轨道平面上的位置
	double x_k = rk * cos(uk);
	double y_k = rk * sin(uk);
	//14.计算改正后的升交点经度
	double OMEGA_k = Eph->OMEGA + (Eph->OMEGADot - Omega_WGS) * dt - Omega_WGS * Eph->TOE.SecOfWeek;
	//15.计算在地固坐标系下的位置
	Mid->SatPos[0] = x_k * cos(OMEGA_k) - y_k * cos(ik) * sin(OMEGA_k);
	Mid->SatPos[1] = x_k * sin(OMEGA_k) + y_k * cos(ik) * cos(OMEGA_k);
	Mid->SatPos[2] = y_k * sin(ik);

	/**********************************计算GPS卫星速度**********************************/
	double EkDot = n / (1 - Eph->e * cos(Ek));
	double upkDot = sqrt(1 - Eph->e * Eph->e) / (1 - Eph->e * cos(Ek)) * EkDot;
	double ukDOT = 2 * (Eph->Cus * cos(2 * upk) - Eph->Cuc * sin(2 * upk)) * upkDot + upkDot;
	double rkDOT = pow(Eph->SqrtA, 2) * Eph->e * sin(Ek) * EkDot + 2 * (Eph->Crs * cos(2 * upk) - Eph->Crc * sin(2 * upk)) * upkDot;
	double ikDOT = Eph->iDot + 2 * (Eph->Cis * cos(2 * upk) - Eph->Cic * sin(2 * upk)) * upkDot;
	double OMEGA_kDOT = Eph->OMEGADot - Omega_WGS;
	double x_kDOT = rkDOT * cos(uk) - rk * ukDOT * sin(uk);
	double y_kDOT = rkDOT * sin(uk) + rk * ukDOT * cos(uk);

	Eigen::MatrixXd R = Eigen::MatrixXd(3, 4);
	R <<cos(OMEGA_k), -sin(OMEGA_k) * cos(ik), -(x_k * sin(OMEGA_k) + y_k * cos(OMEGA_k) * cos(ik)), y_k* sin(OMEGA_k)* sin(ik),
		sin(OMEGA_k), cos(OMEGA_k)* cos(ik), (x_k * cos(OMEGA_k) - y_k * sin(OMEGA_k) * cos(ik)), -y_k * cos(OMEGA_k) * sin(ik),
		0, sin(ik), 0, y_k* cos(ik);
	Eigen::VectorXd vel_vector = Eigen::VectorXd(4);
	vel_vector << x_kDOT, y_kDOT, OMEGA_kDOT, ikDOT;

	Eigen::VectorXd vel_earth = R * vel_vector;
	Mid->SatVel[0] = vel_earth(0);
	Mid->SatVel[1] = vel_earth(1);
	Mid->SatVel[2] = vel_earth(2);

	/**********************************计算GPS卫星钟差**********************************/
	//对钟差进行相对论改正
	double f = -2 * sqrt(GM_Earth) / C_Light / C_Light;
	Mid->SatClkOft += f * Eph->e * Eph->SqrtA * sin(Ek);

	/**********************************计算GPS卫星钟速**********************************/
	Mid->SatClkSft = Eph->ClkDrift + 2 * Eph->ClkDriftRate * ((t->Week - Eph->TOC.Week) * 604800.0 +(t->SecOfWeek - Eph->TOC.SecOfWeek)) + f * Eph->e * Eph->SqrtA * cos(Ek) * EkDot;

	Mid->Valid = true;
	return true;//?
}

/****************************************************************************
  CompBDSSatPVT//在CGCS2000下解算

  目的：计算BDS卫星位置、速度、钟差、钟速（含有相对论改正,但未进行地球自传改正）

  输入参数：
   Prn号，卫星信号发射时刻，单颗卫星星历数据，
  输出参数：
   BDS卫星位置、速度、钟差、钟速（含有相对论改正）
****************************************************************************/
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid)
{
	GPSTIME bdst;
	bdst.Week = t->Week - 1356;
	bdst.SecOfWeek = t->SecOfWeek - 14;

	/**********************************计算BDS卫星位置**********************************/
	//1和2. 计算轨道长半轴，计算平均运动角速度
	double n0 = sqrt(GM_BDS) / pow(Eph->SqrtA, 3);
	//3.计算相对于星历参考历元的时间,注意不要改TOE
	double dt = (bdst.Week - Eph->TOE.Week) * 604800 + (bdst.SecOfWeek - Eph->TOE.SecOfWeek);
	//4. 对平均运动角速度进行改正
	double n = n0 + Eph->DeltaN;
	//5. 计算平近点角
	double Mk = Eph->M0 + n * dt;
	//6. 计算偏近点角（利用下面的开普勒方程，迭代求解）
	double Ek = Mk;
	double previous_Ek = Mk;
	int max_iterations = 100; // 最大迭代次数
	int iteration_count = 0;  // 迭代计数器
	do {
		previous_Ek = Ek;
		Ek = Mk + Eph->e * sin(Ek);
		iteration_count++;
		if (iteration_count > max_iterations) { // 如果达到最大迭代次数则强制跳出循环
			break;
		}
	} while (fabs(Ek - previous_Ek) > 1e-10); // 迭代精度
	//7. 计算真近点角
	double vk = atan2((sqrt(1 - (Eph->e) * (Eph->e)) * sin(Ek)), (cos(Ek) - Eph->e));
	//8. 计算升交角距
	double upk = vk + Eph->omega;
	//9. 计算二阶调和改正数
	double delta_uk = Eph->Cus * sin(2 * upk) + Eph->Cuc * cos(2 * upk);
	double delta_rk = Eph->Crs * sin(2 * upk) + Eph->Crc * cos(2 * upk);
	double delta_ik = Eph->Cis * sin(2 * upk) + Eph->Cic * cos(2 * upk);
	//10.计算经过改正的升交角距
	double uk = upk + delta_uk;
	//11.计算经过改正的向径
	double rk = pow(Eph->SqrtA, 2) * (1 - (Eph->e) * cos(Ek)) + delta_rk;
	//12.计算经过改正的轨道倾角
	double ik = Eph->i0 + delta_ik + Eph->iDot * dt;
	//13.计算卫星在轨道平面上的位置
	double x_k = rk * cos(uk);
	double y_k = rk * sin(uk);

	double OMEGA_k;
	double EkDot, upkDot, ukDOT, rkDOT, ikDOT, OMEGA_kDOT, x_kDOT, y_kDOT;
	Eigen::Vector3d vel_earth;
	Eigen::Vector3d Pos_earth;
	//GEO轨道： 1 2 3 4 5 59 60 61 62 63
	if (Prn <= 5 || (Prn >= 59 && Prn <= 63))
	{
		//14.计算改正后的升交点经度（惯性系）
		OMEGA_k = Eph->OMEGA + Eph->OMEGADot * dt - Omega_BDS * Eph->TOE.SecOfWeek;
		//15.计算GEO卫星在自定义坐标系中的坐标
		double X_GK = x_k * cos(OMEGA_k) - y_k * cos(ik) * sin(OMEGA_k);
		double Y_GK = x_k * sin(OMEGA_k) + y_k * cos(ik) * cos(OMEGA_k);
		double Z_GK = y_k * sin(ik);
		//16.计算GEO卫星在BDCS坐标系中的坐标
		double angle_Rz = Omega_BDS * dt;
		double angle_Rx = -5.0 * PAI / 180.0; 
		Eigen::Matrix3d Rz;
		Rz << cos(angle_Rz), sin(angle_Rz), 0,
			  -sin(angle_Rz), cos(angle_Rz), 0,
			  0, 0, 1;
		Eigen::Matrix3d Rx;
		Rx << 1, 0, 0,
			0, cos(angle_Rx), sin(angle_Rx),
			0, -sin(angle_Rx), cos(angle_Rx);

		Eigen::Vector3d Pos_GK(X_GK, Y_GK, Z_GK);
		Pos_earth = Rz * Rx * Pos_GK;
		Mid->SatPos[0] = Pos_earth(0);
		Mid->SatPos[1] = Pos_earth(1);
		Mid->SatPos[2] = Pos_earth(2);

		/**********************************计算GEO卫星速度**********************************/
		EkDot = n / (1 - Eph->e * cos(Ek));
		upkDot = sqrt(1 - Eph->e * Eph->e) / (1 - Eph->e * cos(Ek)) * EkDot;
		ukDOT = 2 * (Eph->Cus * cos(2 * upk) - Eph->Cuc * sin(2 * upk)) * upkDot + upkDot;
		rkDOT = pow(Eph->SqrtA, 2) * Eph->e * sin(Ek) * EkDot + 2 * (Eph->Crs * cos(2 * upk) - Eph->Crc * sin(2 * upk)) * upkDot;
		ikDOT = Eph->iDot + 2 * (Eph->Cis * cos(2 * upk) - Eph->Cic * sin(2 * upk)) * upkDot;
		OMEGA_kDOT = Eph->OMEGADot;
		x_kDOT = rkDOT * cos(uk) - rk * ukDOT * sin(uk);
		y_kDOT = rkDOT * sin(uk) + rk * ukDOT * cos(uk);

		Eigen::MatrixXd R_GK = Eigen::MatrixXd(3, 4);
		R_GK << cos(OMEGA_k), -sin(OMEGA_k) * cos(ik), -(x_k * sin(OMEGA_k) + y_k * cos(OMEGA_k) * cos(ik)), y_k* sin(OMEGA_k)* sin(ik),
			    sin(OMEGA_k), cos(OMEGA_k)* cos(ik), (x_k * cos(OMEGA_k) - y_k * sin(OMEGA_k) * cos(ik)), -y_k * cos(OMEGA_k) * sin(ik),
			    0, sin(ik), 0, y_k* cos(ik);
		Eigen::VectorXd vel_vector_GK = Eigen::VectorXd(4);
		vel_vector_GK << x_kDOT, y_kDOT, OMEGA_kDOT, ikDOT;
		Eigen::VectorXd vel_GK = R_GK * vel_vector_GK;

		Eigen::Matrix3d Rz_DOT;//求导可得，如果报错可以检查一下这里
		Rz_DOT << -Omega_BDS * sin(angle_Rz), Omega_BDS* cos(angle_Rz), 0,
			      -Omega_BDS * cos(angle_Rz), -Omega_BDS * sin(angle_Rz), 0,
			      0, 0, 0;

		vel_earth = Rz * Rx * vel_GK + Rz_DOT * Rx * Pos_GK;
		Mid->SatVel[0] = vel_earth(0);
		Mid->SatVel[1] = vel_earth(1);
		Mid->SatVel[2] = vel_earth(2);

	}
	else 
	{
		//14.计算改正后的升交点经度（地固系）
		OMEGA_k = Eph->OMEGA + (Eph->OMEGADot - Omega_BDS) * dt - Omega_BDS * Eph->TOE.SecOfWeek;
		//15.计算MEO/IGSO卫星在BDCS坐标系中的坐标
		Mid->SatPos[0] = x_k * cos(OMEGA_k) - y_k * cos(ik) * sin(OMEGA_k);
		Mid->SatPos[1] = x_k * sin(OMEGA_k) + y_k * cos(ik) * cos(OMEGA_k);
		Mid->SatPos[2] = y_k * sin(ik);

		/**********************************计算MEO/IGSO卫星速度**********************************/
		EkDot = n / (1 - Eph->e * cos(Ek));
		upkDot = sqrt(1 - Eph->e * Eph->e) / (1 - Eph->e * cos(Ek)) * EkDot;
		ukDOT = 2 * (Eph->Cus * cos(2 * upk) - Eph->Cuc * sin(2 * upk)) * upkDot + upkDot;
		rkDOT = pow(Eph->SqrtA, 2) * Eph->e * sin(Ek) * EkDot + 2 * (Eph->Crs * cos(2 * upk) - Eph->Crc * sin(2 * upk)) * upkDot;
		ikDOT = Eph->iDot + 2 * (Eph->Cis * cos(2 * upk) - Eph->Cic * sin(2 * upk)) * upkDot;
		OMEGA_kDOT = Eph->OMEGADot - Omega_BDS;
		x_kDOT = rkDOT * cos(uk) - rk * ukDOT * sin(uk);
		y_kDOT = rkDOT * sin(uk) + rk * ukDOT * cos(uk);

		Eigen::MatrixXd R = Eigen::MatrixXd(3, 4);
		R << cos(OMEGA_k), -sin(OMEGA_k) * cos(ik), -(x_k * sin(OMEGA_k) + y_k * cos(OMEGA_k) * cos(ik)), y_k* sin(OMEGA_k)* sin(ik),
			sin(OMEGA_k), cos(OMEGA_k)* cos(ik), (x_k * cos(OMEGA_k) - y_k * sin(OMEGA_k) * cos(ik)), -y_k * cos(OMEGA_k) * sin(ik),
			0, sin(ik), 0, y_k* cos(ik);
		Eigen::VectorXd vel_vector = Eigen::VectorXd(4);
		vel_vector << x_kDOT, y_kDOT, OMEGA_kDOT, ikDOT;

		vel_earth = R * vel_vector;
		Mid->SatVel[0] = vel_earth(0);
		Mid->SatVel[1] = vel_earth(1);
		Mid->SatVel[2] = vel_earth(2);

	}

	/**********************************计算BDS卫星钟差**********************************/
	//对钟差进行相对论改正，卫星设备延迟改正TGD
	double f = -2 * sqrt(GM_BDS) / C_Light / C_Light;
	Mid->SatClkOft += f * Eph->e * Eph->SqrtA * sin(Ek);
	Mid->Tgd1 = Eph->TGD1;
	Mid->Tgd2 = Eph->TGD2;

	/**********************************计算BDS卫星钟速**********************************/
	Mid->SatClkSft = Eph->ClkDrift + 2 * Eph->ClkDriftRate * ((t->Week - Eph->TOC.Week) * 604800.0 + (t->SecOfWeek - Eph->TOC.SecOfWeek)) + f * Eph->e * Eph->SqrtA * cos(Ek) * EkDot;

	Mid->Valid = true;
	return true;
}

/****************************************************************************
  RotationCorrection

  目的：地球自转改正

  输入参数：
	卫星的位置和速度，用户位置，自转角速度
  输出参数：
	经地球自转改正后的卫星位置和速度
  容错处理：

****************************************************************************/
void RotationCorrection(SATMIDRES* Mid,double UserPos[3],double Omega_e)
{
	double rou, dt;
	rou = sqrt((Mid->SatPos[0] - UserPos[0]) * (Mid->SatPos[0] - UserPos[0]) + (Mid->SatPos[1] - UserPos[1]) * (Mid->SatPos[1] - UserPos[1]) + (Mid->SatPos[2] - UserPos[2]) * (Mid->SatPos[2] - UserPos[2]));
	dt = rou / C_Light;

	double angle_Rz = Omega_e * dt;
	Eigen::Matrix3d Rz;
	Rz << cos(angle_Rz), sin(angle_Rz), 0,
		 -sin(angle_Rz), cos(angle_Rz), 0,
		  0, 0, 1;

	Eigen::Vector3d Pos(Mid->SatPos[0], Mid->SatPos[1], Mid->SatPos[2]);
	Eigen::Vector3d Vel(Mid->SatVel[0], Mid->SatVel[1], Mid->SatVel[2]);
	Pos= Rz * Pos;
	Mid->SatPos[0] = Pos(0);
	Mid->SatPos[1] = Pos(1);
	Mid->SatPos[2] = Pos(2);
	Vel= Rz * Vel;
	Mid->SatVel[0] = Vel(0);
	Mid->SatVel[1] = Vel(1);
	Mid->SatVel[2] = Vel(2);

}


