#include "SPP.h"
extern ROVERCFGINFO cfginfo;

/****************************************************************************
  解算当前历元SPP

  目的：单点定位

  输入参数：
	观测数据的结构体，卫星星历
  输出参数：
	用户定位结果
  容错处理：
	少于4颗卫星不进行定位解算
****************************************************************************/
bool SPP(EPOCHOBSDATA* obs, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res)
{
	double R, F;//半径和扁率
	double blh[3];//测站大地坐标

	//① 设定初始位置𝑋𝑅，可以直接放到结构体里赋值
	//Res->Pos[0] = -2267335.9727; Res->Pos[1] = 5008647.9932; Res->Pos[2] = 3222372.6377; Res->RcvClkOft[0] = Res->RcvClkOft[0] = 0.0;
	Res->Pos[0] = 0.1; Res->Pos[1] = 0.1; Res->Pos[2] = 0.1;Res->RcvClkOft[0] = Res->RcvClkOft[0] = 0.0;
	Res->RcvClkOft[0] = Res->RcvClkOft[1] = 0.0;
	//double initialPos[3] = { Res->Pos[0], Res->Pos[1], Res->Pos[2] };//保留初始位置

	const int maxIterations = 15;
	const double convergenceThreshold = 1e-8;
	int iteration = 0;
	bool converged = false;

	while (!converged && iteration < maxIterations)
	{
		//② 计算信号发射时刻的卫星位置和钟差(地球自转改正)，计算对流层延迟等
		SatellitePosition(GPSEph, BDSEph, obs, Res->Pos);//Res->Pos的输入有问题吗
		for (int t = 0; t < obs->SatNum; t++)
		{
			/*状态不好的卫星不参与解算、卫星位置解算失败也不参与解算*/
			if ((!obs->SatObs[t].Valid || !obs->SatPVT[t].Valid)) continue;
			/*计算卫星高度角*/
			if (obs->SatObs[t].System == GPS) { R = R_WGS84; F = F_WGS84; }
			else if (obs->SatObs[t].System == BDS) { R = R_CGS2K; F = F_CGS2K; }
			else continue;
			CompSatElAz(Res->Pos, obs->SatPVT[t].SatPos, R, F, &obs->SatPVT[t].Elevation, &obs->SatPVT[t].Azimuth);
			//if ((obs->SatPVT[t].Elevation < 0) || (obs->SatPVT[t].Elevation > (PAI / 2))) { obs->SatPVT[t].Valid = false; continue; }
			//短基线进行高度角设置
			if ((obs->SatPVT[t].Elevation < (cfginfo.ElevThreshold * Rad) ) || (obs->SatPVT[t].Elevation > (PAI / 2))) { obs->SatPVT[t].Valid = false; continue; }
			/*计算对流层延迟*/
			XYZToBLH(Res->Pos, blh, R, F);
			obs->SatPVT[t].TropCorr = hopfield(blh[2], obs->SatPVT[t].Elevation);
		}

		// 保存上次的位置和钟差
		double prevPos[3] = { Res->Pos[0], Res->Pos[1], Res->Pos[2] };
		double prevClkOft[2] = { Res->RcvClkOft[0], Res->RcvClkOft[1] };

		//③ 利用最小二乘办法求解，并计算精度
		if (!SPP_LS(obs, Res))
		{
			return false;
		}

		//④  检查收敛
		//位置变化
		double deltaNorm = 0.0;
		for (int i = 0; i < 3; i++) {
			double delta = Res->Pos[i] - prevPos[i];
			deltaNorm += delta * delta;
		}

		// 钟差变化
		double clkDelta = 0.0;
		clkDelta += pow(Res->RcvClkOft[0] - prevClkOft[0], 2);
		clkDelta += pow(Res->RcvClkOft[1] - prevClkOft[1], 2);
		deltaNorm = sqrt(deltaNorm + clkDelta);

		converged = (deltaNorm < convergenceThreshold);
		iteration++;
	}
	Res->SatNum = obs->SatNum;
	Res->Time = obs->Time;
	return true;
}

/****************************************************************************
  解算当前历元SPV

  目的：单点测速

  输入参数：
	观测数据的结构体
  输出参数：
	用户定位结果
  容错处理：
	
****************************************************************************/
bool SPV(EPOCHOBSDATA* obs, POSRES* Res)
{
	int i, j, m;
	double rou;

	//1.构建矩阵B和w
	int cols = 4;
	int validSatCount = 0;
	for (m = 0; m < obs->SatNum; m++) {
		if ((!obs->SatObs[m].Valid || !obs->SatPVT[m].Valid)) continue;
		validSatCount++;
	}
	if (validSatCount < 4) {
		return false; // 卫星数不足未知参数数量
	}

	// 初始化矩阵
	MatrixXd B = MatrixXd::Zero(validSatCount, cols);
	VectorXd w = VectorXd::Zero(validSatCount);
	MatrixXd P = MatrixXd::Identity(validSatCount, validSatCount);
	int index = 0;
	for (i = 0; i < obs->SatNum; i++)
	{
		// 跳过无效卫星
		if (!obs->SatPVT[i].Valid) continue;
		if (!obs->SatObs[i].Valid) continue;

		rou = sqrt((obs->SatPVT[i].SatPos[0] - Res->Pos[0]) * (obs->SatPVT[i].SatPos[0] - Res->Pos[0])
			+ (obs->SatPVT[i].SatPos[1] - Res->Pos[1]) * (obs->SatPVT[i].SatPos[1] - Res->Pos[1])
			+ (obs->SatPVT[i].SatPos[2] - Res->Pos[2]) * (obs->SatPVT[i].SatPos[2] - Res->Pos[2]));
		if (rou < 1e-3) continue;
		B(index, 0) = (Res->Pos[0] - obs->SatPVT[i].SatPos[0]) / rou;
		B(index, 1) = (Res->Pos[1] - obs->SatPVT[i].SatPos[1]) / rou;
		B(index, 2) = (Res->Pos[2] - obs->SatPVT[i].SatPos[2]) / rou;
		B(index, 3) = 1.0;
		w(index) = obs->SatObs[i].D[0] - ((obs->SatPVT[i].SatPos[0] - Res->Pos[0]) * obs->SatPVT[i].SatVel[0] + (obs->SatPVT[i].SatPos[1] - Res->Pos[1]) * obs->SatPVT[i].SatVel[1] + (obs->SatPVT[i].SatPos[2] - Res->Pos[2]) * obs->SatPVT[i].SatVel[2]) / rou + C_Light * obs->SatPVT[i].SatClkSft;
		
		index++;

	}

	//2.最小二乘求解
	MatrixXd N = B.transpose() * P * B;
	VectorXd W = B.transpose() * P * w;
	VectorXd x;
	x = N.inverse() * W;
	/*if (N.determinant() > 1e-6) {
		x = N.ldlt().solve(W);
	}
	else {
		JacobiSVD<MatrixXd> svd(N, ComputeThinU | ComputeThinV);
		x = svd.solve(W);
	}*/
	double dPos[3] = { x(0), x(1), x(2) };
	for (j = 0; j < 3; j++) {
		Res->Vel[j] = dPos[j];
	}
	Res->RcvClkSft = x(3);

	//3.精度评定
	VectorXd v = B * x - w;
	int dof = validSatCount - cols; // 自由度
	if (dof > 0) {
		Res->SigmaVel = sqrt((v.transpose() * v)(0) / dof); // 单位权重下简化为v'v
	}
	else {
		Res->SigmaVel = 0.0;
	}
	return true;
}

/****************************************************************************
  SPP_LS

  目的：最小二乘解算

  输入参数：
	观测数据的结构体
  输出参数：
	用户定位结果
  容错处理：

****************************************************************************/
bool SPP_LS(EPOCHOBSDATA* obs, POSRES* Res)
{
	int i,j,m;
	Res->GPS_SatNum = Res->BDS_SatNum = 0;
	const int maxCols = 5; // 最大列数（双系统）
	int cols = maxCols;
	double rou;

	//1. 对所有卫星的观测数据进行线性化
		//a.卫星位置计算失败、观测数据不完整或有粗差，不参与定位计算
		//b.以初始位置为参考，对观测方程线性化，计算观测系数矩阵𝐵𝑛∗5和残差向量𝑤(𝑎 + 𝑏) ∗1，统计参与定位的各系统卫星数和所有卫星数
	//2. 卫星总数是否大于未知参数数量，如果卫星数不足，直接返回定位失败。
	int validSatCount = 0;
	for (m = 0; m < obs->SatNum; m++) {
		if ((!obs->SatObs[m].Valid || !obs->SatPVT[m].Valid)) continue;
		validSatCount++;
	}
	if (validSatCount < maxCols) {
		return false; // 卫星数不足未知参数数量
	}

	// 初始化矩阵
	MatrixXd B = MatrixXd::Zero(validSatCount, maxCols);
	VectorXd w = VectorXd::Zero(validSatCount);
	MatrixXd P = MatrixXd::Identity(validSatCount, validSatCount);

	//观测方程线性化
	int index = 0;
	for (i = 0; i < obs->SatNum; i++) {
		// 跳过无效卫星
		if (!obs->SatPVT[i].Valid) continue;
		if (!obs->SatObs[i].Valid) continue;

		rou = sqrt((obs->SatPVT[i].SatPos[0] - Res->Pos[0]) * (obs->SatPVT[i].SatPos[0] - Res->Pos[0])
			+ (obs->SatPVT[i].SatPos[1] - Res->Pos[1]) * (obs->SatPVT[i].SatPos[1] - Res->Pos[1])
			+ (obs->SatPVT[i].SatPos[2] - Res->Pos[2]) * (obs->SatPVT[i].SatPos[2] - Res->Pos[2]));
		if (rou < 1e-3) continue;

		// 构建设计矩阵B
		B(index, 0) = (Res->Pos[0] - obs->SatPVT[i].SatPos[0]) / rou;
		B(index, 1) = (Res->Pos[1] - obs->SatPVT[i].SatPos[1]) / rou;
		B(index, 2) = (Res->Pos[2] - obs->SatPVT[i].SatPos[2]) / rou;

		if (obs->SatObs[i].System == GPS) {
			Res->GPS_SatNum++;
			B(index, 3) = 1.0; // GPS钟差
			w(index) = obs->ComObs[i].PIF - (rou - C_Light * obs->SatPVT[i].SatClkOft + obs->SatPVT[i].TropCorr + Res->RcvClkOft[0]);
		}
		else if (obs->SatObs[i].System == BDS) {
			Res->BDS_SatNum++;
			B(index, 4) = 1.0; // BDS钟差
			w(index) = obs->ComObs[i].PIF - C_Light * obs->SatPVT[i].Tgd1* (FG1_BDS * FG1_BDS / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS)) - (rou - C_Light * obs->SatPVT[i].SatClkOft + obs->SatPVT[i].TropCorr + Res->RcvClkOft[1]);
		}
		index++;
	}

	//3. 如果GPS或BDS卫星数量为0，重构法方程矩阵N和Y
	const bool dualSystem = (Res->GPS_SatNum > 0 && Res->BDS_SatNum > 0);
	if (!dualSystem) {
		cols = 4; // 单系统使用4列
		// 移除未使用的钟差列
		MatrixXd newB(validSatCount, cols);
		if (Res->GPS_SatNum > 0) {
			newB << B.leftCols(4);
		}
		else if (Res->BDS_SatNum > 0) {
			newB << B.leftCols(3), B.col(4);
		}
		B.resize(validSatCount, 4);
		B = newB;
	}

	//4. 最小二乘求解 
	MatrixXd N = B.transpose() * P * B;
	VectorXd W = B.transpose() * P * w;
	VectorXd x;
	x = N.inverse() * W;
	/*if (N.determinant() > 1e-6) {
		x = N.ldlt().solve(W);
	}
	else {
		JacobiSVD<MatrixXd> svd(N, ComputeThinU | ComputeThinV);
		x = svd.solve(W);
	}*/

	//更新接收机位置
	
	for (j = 0; j < 3; j++) {
		Res->Pos[j] += x(j);
	}

	// 更新钟差
	if (dualSystem) {
		Res->RcvClkOft[0] += x(3);
		Res->RcvClkOft[1] += x(4);
	}
	else if (Res->GPS_SatNum > 0) {
		Res->RcvClkOft[0] += x(3);
	}
	else if (Res->BDS_SatNum > 0) {
		Res->RcvClkOft[1] += x(3);
	}

	//5. 定位精度评价，计算PDOP和标准差
	VectorXd v = B * x - w;
	int dof = validSatCount - cols; // 自由度
	if (dof > 0) {
		Res->SigmaPos = sqrt((v.transpose() * v)(0) / dof); // (0) 的作用：访问该1x1矩阵的唯一元素（位置索引为0），将其转换为标量值（如 double 类型）
	}
	else {
		Res->SigmaPos = 0.0;
	}

	MatrixXd Q = N.inverse();
	Res->PDOP = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));

	return true;
}
