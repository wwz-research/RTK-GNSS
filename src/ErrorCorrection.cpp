#include "SPP.h"

/****************************************************************************
  DetectOutlier

  目的：粗差探测

  输入参数：obs  每个历元观测数据
   
****************************************************************************/
void DetectOutlier(EPOCHOBSDATA* obs)
{

	/*一个历元中有多颗卫星，通过循环对其中一颗卫星进行粗差探测。
	函数内部可先定义MWGF CurComObs[MAXCHANNUM]; 用于存放当前历元的计算结果，EPOCHOBSDATA结构体中ComObs保存上个历元的平滑结果。*/
	MWGF CurComObs[MAXCHANNUM];
	int i, j;
	double f[2];
	bool Status = false;//用于判断是否找到任意一颗卫星在上个历元中的对应卫星
	double dGF = 0.0;
	double dMW = 0.0;

	//对于任意一颗卫星的观测数据，其粗差探测的步骤为
	for (i = 0; i < obs->SatNum; i++)
	{ 
		//1. 检查该卫星的双频伪距和相位数据是否有效和完整，若不全或为0，将Valid标记为false，continue
		CurComObs[i].Sys = obs->SatObs[i].System;
		CurComObs[i].Prn = obs->SatObs[i].Prn;

		if (fabs(obs->SatObs[i].P[0]) < 1e-10 ||
			fabs(obs->SatObs[i].P[1]) < 1e-10 ||
			fabs(obs->SatObs[i].L[0]) < 1e-10 ||
			fabs(obs->SatObs[i].L[1]) < 1e-10)
		{
			obs->SatObs[i].Valid = false;
			continue; // 跳过无效数据
		}



		//2. 计算当前历元该卫星的GF和MW组合值
		if (CurComObs[i].Sys == GPS)
		{
			f[0] = FG1_GPS;
			f[1] = FG2_GPS;
		}
		else if (CurComObs[i].Sys == BDS)
		{
			f[0] = FG1_BDS;
			f[1] = FG3_BDS;
		}
		else continue;
		CurComObs[i].GF = obs->SatObs[i].L[0] - obs->SatObs[i].L[1];
		CurComObs[i].MW = 1 / (f[0] - f[1]) * (f[0] * obs->SatObs[i].L[0] - f[1] * obs->SatObs[i].L[1])
			- 1 / (f[0] + f[1]) * (f[0] * obs->SatObs[i].P[0] + f[1] * obs->SatObs[i].P[1]);

		//3. 从上个历元的MWGF数据中查找该卫星的GF和MW组合值,怎么设置循环最合理呢？
		for (j = 0; j < MAXCHANNUM; j++)
		{
			if (obs->ComObs[j].Prn == 0) continue;
			if (CurComObs[i].Sys != obs->ComObs[j].Sys) continue;
			if (CurComObs[i].Prn != obs->ComObs[j].Prn ) continue;
			Status = true;

			//4. 计算当前历元该卫星GF与上一历元对应GF的差值dGF
			dGF = CurComObs[i].GF - obs->ComObs[j].GF;
			//5. 计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
			dMW = CurComObs[i].MW - obs->ComObs[j].MW;

			break; // 找到对应卫星后退出循环
		}

		//6. 检查dGF和dMW是否超限，限差阈值建议为5cm和3m。若超限，标记为粗差，将Valid标记为false ，若不超限，标记为可用将Valid标记为true，并计算该卫星的MW平滑值
		if (Status)
		{
			if (fabs(dGF) > 0.05) // 5cm 阈值
			{
				obs->SatObs[i].Valid = false;
			}
			else if (fabs(dMW) > 3.0) // 3m 阈值
			{
				obs->SatObs[i].Valid = false;
			}
			else
			{
				obs->SatObs[i].Valid = true;
				// 计算 MW 平滑值
				CurComObs[i].n = obs->ComObs[j].n + 1;
				CurComObs[i].MW = (CurComObs[i].MW + obs->ComObs[j].MW * obs->ComObs[j].n) / CurComObs[i].n;
			}
		}
		else
		{
			// 如果没有找到对应卫星，该颗卫星标记为无效,且设置为第一次对这个卫星的观测
			obs->SatObs[i].Valid = true ;//(OMG，错了吧）
			CurComObs[i].n = 1;
		}

		// 7. 对于可用的观测数据，计算伪距的IF组合观测值，用于SPP
		CurComObs[i].PIF = (pow(f[0], 2) * obs->SatObs[i].P[0] - pow(f[1], 2) * obs->SatObs[i].P[1]) / (pow(f[0], 2) - pow(f[1], 2));
	}

	// 8. 所有卫星循环计算完成之后，将CurComObs内存拷贝到ComObs，即函数运行结束后， ComObs保存了当前历元的GF和MW平滑值。
	memcpy(obs->ComObs, CurComObs, sizeof(MWGF)* MAXCHANNUM);

}

/****************************************************************************
  hopfield

  目的：对流层改正

  输入参数：
	测站高度、高度角
  输出参数：
	返回对流层改正值
  容错处理：
	测站高度不在对流层范围内，直接输出为0
****************************************************************************/
double hopfield(double H, double Elev)
{
	if (fabs(H) >= 20000) return 0;
	double trop;
	double H0 = 0.0;//海平面
	double T0 = 15 + 273.16;//温度
	double P0 = 1013.25;//气压
	double RH0 = 0.5;//相对湿度

	//Hopefield模型
	double RH = RH0 * exp(-0.0006396 * (H - H0));
	double P = P0 * pow(1 - 0.0000226 * (H - H0), 5.225);
	double T = T0 - 0.0065 * (H - H0);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double hw = 11000;
	double hd = 40136 + 148.72 * (T0 - 273.16);
	double Kw = 155.2e-7 * 4810 / (pow(T, 2)) * e * (hw - H);
	double Kd = 155.2e-7 * P / T * (hd - H);
	trop = Kd / sin(sqrt(pow(Elev * 180 / PAI, 2) + 6.25) * PAI / 180) + Kw / sin(sqrt(pow(Elev * 180 / PAI, 2) + 2.25) * PAI / 180);

	return trop;
}
