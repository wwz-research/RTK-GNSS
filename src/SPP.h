#pragma once

#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<cmath>
#include<sstream>
#include<iomanip>
#include <Eigen/Dense>
#include"Socket.h"

using namespace std;
using namespace Eigen;

//#define _CRT_SECURE_NO_WARNINGS

#define PAI 3.1415926535898
#define PAI2 (2.0*PAI)                   /* 2pi */
#define Rad (PAI/180.0)                  /* Radians per degree */
#define Deg (180.0/PAI)                  /* Degrees per radian */
#define C_Light 299792458.0              /* Speed of light  [m/s]; IAU 1976  */

#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate地球自转角速度 */ 
#define GM_Earth   398600.5e+9      /* [m^3/s^2]; WGS-84 */
#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */
#define Omega_BDS 7.2921150e-5      /*[rad/s], the earth rotation rate */
#define GM_BDS   398600.4418e+9     /* [m^3/s^2]; CGCS2000  */

/* some constants about GPS satellite signal */
#define  FG1_GPS  1575.42E6             /* L1信号频率 */
#define  FG2_GPS  1227.60E6             /* L2信号频率 */
#define  FG12R    (77/60.0)             /* FG1_Freq/FG2_Freq */
#define  FG12R2   (5929/3600.0)
#define  WL1_GPS  (C_Light/FG1_GPS)
#define  WL2_GPS  (C_Light/FG2_GPS)

/* some constants about Compass satellite signal */
#define  FG1_BDS  1561.098E6               /* B1信号的基准频率 */
#define  FG2_BDS  1207.140E6               /* B2信号的基准频率 */
#define  FG3_BDS  1268.520E6               /* B3信号的基准频率 */
#define  FC12R    (FG1_BDS/FG2_BDS)       /* FG1_BDS/FG2_BDS */
#define  FC12R2   (FC12R*FC12R)           /* FG1_BDS^2/FG2_BDS^2 */
#define  FC13R    (FG1_BDS/FG3_BDS)       /* FG1_BDS^2/FG3_BDS^2 */
#define  FC13R2   (FC13R*FC13R)
#define  WL1_BDS  (C_Light/FG1_BDS)                                                                                                                                              
#define  WL2_BDS  (C_Light/FG2_BDS)
#define  WL3_BDS  (C_Light/FG3_BDS)

#define MAXGPSNUM  32
#define MAXBDSNUM 63
#define MAXCHANNUM 36               //一个历元最多的可视卫星数？
#define MAXRAWLEN 40960             //缓冲区大小
#define POLYCRC32 0xEDB88320u       //CRC校验码除数

//#define FILEMODE 1                         //1：二进制文件；0：串口
//#define RTKProcMode 1                    //1：EKF最小二乘；2：最小二乘    改为从配置文件读入

//定义导航卫星系统 
enum GNSSSys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };   //UNKS未定义系统

//定义通用时结构体
struct COMMONTIME {
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;

	COMMONTIME()
	{
		Year = 0;
		Month = 0;
		Day = 0;
		Hour = 0;
		Minute = 0;
		Second = 0.0;
	}
};

//定义简化儒略日结构体
struct MJDTIME {
	int Days;//整数天
	double FracDay;//小数天

	MJDTIME()
	{
		Days = 0;
		FracDay = 0.0;
	}
};

//定义GPS时结构体
struct GPSTIME {
	unsigned short Week;
	double SecOfWeek;

	GPSTIME()
	{
		Week = 0;
		SecOfWeek = 0.0;
	}
};

//卫星广播星历
struct GPSEPHREC {
	short Prn;
	GNSSSys Sys;
	GPSTIME TOC, TOE;                             //卫星钟参考时刻，星历（轨道）参考时刻（s）
	double ClkBias, ClkDrift, ClkDriftRate;       //卫星钟的偏差(s)、漂移、漂移速度
	double IODE, IODC;                            //星历发布时间，钟的数据龄期
	double SqrtA, M0, e, OMEGA, i0, omega;        //轨道长半径平方根(根号m)，平近点角(rad)，偏心率，升交点赤经(rad)，轨道倾角(rad)，近地点角距(rad)
	double Crs, Cuc, Cus, Cic, Cis, Crc;          //短周期摄动项系数(m,rad,rad,rad,rad,m)
	double DeltaN, OMEGADot, iDot;                //平均角速度改正数（rad/s),长期摄动项参数（rad/s)，iDot（rad/s)
	int SVHealth;              //BDS:0 is good
	double TGD1, TGD2;         //群延迟（s,GPS一个，BDS两个）

	GPSEPHREC() {
		Prn = 0;
		SVHealth = 1;
		Sys = UNKS;
		ClkBias = ClkDrift = ClkDriftRate = IODE = IODC = TGD1 = TGD2 = 0.0;
		SqrtA = e = M0 = OMEGA = i0 = omega = OMEGADot = iDot = DeltaN = 0.0;
		Crs = Cuc = Cus = Cic = Cis = Crc = 0.0;
	}
};

//定义每颗卫星的观测数据
struct SATOBSDATA {
	short Prn;
	GNSSSys System;
	double P[2], L[2], D[2];
	double cn[2], LockTime[2];  //载噪比，锁定时长
	unsigned char half[2];      //半周状态
	bool Valid;                 //标记值（粗差探测，检查该卫星的双频伪距和相位数据是否有效和完整，若不全或为0，将Valid标记为false，continue）

	SATOBSDATA()
	{
		Prn = 0;
		System = UNKS;
		for(int i=0;i<2;i++)
		{
			P[i] = L[i] = D[i] = 0.0;
		}
		Valid = false;
	}
};

// 每颗卫星位置、速度、钟差和钟速等的中间计算结果 
struct SATMIDRES{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;  //false=没有星历或星历过期,true=计算成功（卫星定位，卫星钟差定义中计算）

	SATMIDRES()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = PAI / 2.0;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};

struct MWGF{
	short Prn;//卫星号
	GNSSSys Sys;
	double MW;
	double GF;
	double PIF;
	int n; //平滑计数

	MWGF()
	{
		Prn = n = 0;
		Sys = UNKS;
		MW = GF = PIF = 0.0;
	}
};

//定义每个历元的观测数据
struct EPOCHOBSDATA {
	GPSTIME Time;      //从头文件中获取
	short SatNum;
	SATOBSDATA SatObs[MAXCHANNUM];
	MWGF ComObs[MAXCHANNUM];
	SATMIDRES  SatPVT[MAXCHANNUM];//ComObs和SatPVT数组存储的卫星顺序，与SatObs数组相同，即用相同循环i，可以找到该卫星的观测值、卫星位置和可用性
	double     Pos[3];      // 保存基站或NovAtel接收机定位结果

	EPOCHOBSDATA()
	{
		SatNum = 0;
		Pos[0] = Pos[1] = Pos[2] = 0.0;
	}
};

//定义每个历元SPP的定位结果结构体
struct POSRES {
	GPSTIME Time;
	double Pos[3], Vel[3];
	double RcvClkOft[2];//接收机GPS、BDS信号钟差
	double RcvClkSft;//接收机钟差变化率
	double PDOP, SigmaPos, SigmaVel;        //精度指标
	int SatNum, GPS_SatNum, BDS_SatNum;
	bool   IsSuccess;                //单点定位是否成功, 1为成功, 0为失败

	POSRES()
	{
		for (int i = 0; i < 3; i++)
		{
			Pos[i] = Vel[i] = 0.0;
		}
		RcvClkOft[0] = RcvClkOft[1] = 0.0;
		PDOP = SigmaPos = SigmaVel = 999.9;
		GPS_SatNum = BDS_SatNum = SatNum = 0;
		IsSuccess = false;
	}
};

//  每颗卫星的单差观测数据定义 
struct SDSATOBS{
	short    Prn;
	GNSSSys  System;
	bool    Valid;          
	double   dP[2], dL[2];   // 单位：m!
	short    nBas, nRov;     // 存储单差观测值对应的基准和流动站的数值索引号
	unsigned char half[2];   // 单差半周状态

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKS;
		dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
		Valid = false;
	}
};

//  每个历元的单差观测数据定义 
struct SDEPOCHOBS{
	GPSTIME    Time;   //根据需求，暂更关注流动站
	short      SatNum; //单差观测值数量
	SDSATOBS   SdSatObs[MAXCHANNUM];
	MWGF       SdCObs[MAXCHANNUM];

	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};

//  双差相关的数据定义
struct DDCOBS{
	int RefPrn[2], RefPos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
	double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
	float  FixRMS[2];                 // 固定解定位中rms误差
	double PDOP;
	double dPos[3];                   // 固定解基线向量
	double dPos_float[3];             // 浮点解基线向量
	double RovPos[3];                 // 固定解流动站位置,bestpos_base+dPos
	double RovPos_float[3];           // 浮点解流动站位置,bestpos_base+dPos_float

	bool bFixed;                      // true为固定，false为未固定

	DDCOBS()
	{
		int i;
		for (i = 0; i < 2; i++) {
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
		dPos[0] = dPos[1] = dPos[2] = dPos_float[0] = dPos_float[1] = dPos_float[2] = 0.0;
		RovPos[0] = RovPos[1] = RovPos[2] = RovPos_float[0] = RovPos_float[1] = RovPos_float[2] = 0.0;
		ResAmb[0] = ResAmb[1] = FixRMS[0] = FixRMS[1] = PDOP = Ratio = 0.0;
		bFixed = false;
		for (i = 0; i < MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
	}
};


//RTK浮点解数据定义，用于存储相对定位浮点解函数的输出参数（最小二乘）：主要是用于Q矩阵在浮点解函数和Lambda函数之间的传递，因为浮点解、模糊度固定、固定解不是在同一个函数
struct FloatResult {
	double dX[3];                           // 基线向量（dx,dy,dz）Rov-Bas
	double N[MAXCHANNUM * 2];               // 双差模糊度
	double sigma;                           // 验后单位权中误差
	double DOP[3];                          // 基线向量对应的协因数（Qxx,Qyy,Qzz）
	//double PDOP;
	double Q[(MAXCHANNUM * 2) * (MAXCHANNUM * 2)]; // 模糊度的协因数矩阵 ，注意大小
	int stanum[2], totalsatnum; // 解算所用双差卫星数（不包括两个系统各自的基准星）-> 用于确定Q的有效元素数量

	FloatResult()
	{
		dX[0] = dX[1] = dX[2] = 0.0;
		DOP[0] = DOP[1] = DOP[2] = 0.0;
		//PDOP = 0.0;
		sigma = 0.0;
		stanum[0] = stanum[1] = totalsatnum = 0;
		for (int i = 0; i < MAXCHANNUM * 2; i++)
		{
			N[i] = 0.0;
			for (int j = 0; j < MAXCHANNUM * 2; j++)
			{
				Q[i * MAXCHANNUM * 2 + j] = 0.0;
			}
		}
	}
};

//卡尔曼滤波数据定义
struct RTKEKF {
	GPSTIME Time;
	//待估参数：Rover位置参数，双差模糊度；协方差P，这里不是权阵。X0、P0为状态备份
	Eigen::Vector<double, 3 + MAXCHANNUM * 2> X, X0;              
	Eigen::Matrix<double, 3 + MAXCHANNUM * 2, 3 + MAXCHANNUM * 2> P, P0;
	//int Index[MAXCHANNUM]; // index存放的是上一历元SDOBS的每个数据对应在X_k-1中的索引
	int nSats;
	DDCOBS DDObs, CurDDObs;  // 上一个历元和当前历元的双差观测值信息
	SDEPOCHOBS SDObs;        // 上一个历元的单差观测值
	bool IsInit;             // 滤波是否初始化

	RTKEKF() 
	{
		nSats = 0;
		IsInit = false;
		/*for (int i = 0; i < MAXCHANNUM; i++) 
		{
			Index[i] = -1;
		}*/

		X.setZero();
		X0.setZero();
		P.setZero();
		P0.setZero();
	}
};

//RTK定位的数据定义
struct RAWDAT {
	EPOCHOBSDATA BasEpk;    //基站每个历元的观测数据
	EPOCHOBSDATA RovEpk;    //流动站每个历元的观测数据
	EPOCHOBSDATA BasEpk0;   //基站上个历元的观测数据
	EPOCHOBSDATA RovEpk0;   //流动站上个历元的观测数据
	SDEPOCHOBS SdObs;       //每个历元的单差观测数据定义
	DDCOBS DDObs;           //双差相关的数据定义，存储固定解
	GPSEPHREC GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];        //星历数据
	POSRES bestpos_base;    //基站坐标
	POSRES bestpos_rover;   //流动站坐标，实际无法解得流动站bestpos坐标
};

// 配置参数结构体
struct ROVERCFGINFO
{
	short IsFileData, RTKProMode;                    //1=FILE,0=COM;1=EKF,2=LSQ
	int RovPort, RovBaud;                            //移动站串口号、波特率
	char BasNetIP[20], RovNetIP[20];                 //ip address
	short BasNetPort, RovNetPort;                    //port 基站/移动站端口号
	double CodeNoise, CPNoise;                       //伪距噪声，载波相位噪声
	double ElevThreshold;                            //高度角阈值
	double RatioThres;                               //Ratio阈值
	
	char BasObsDatFile[256], RovObsDatFile[256];          //观测数据文件名
	char ResFile[256];                               //结果数据文件名
	char DiffFile[256];                              //定位结果和接收机位置插值文件名

	ROVERCFGINFO()
	{
		IsFileData = RTKProMode = 1;
		RovPort = RovBaud = BasNetPort = RovNetPort = 0;
		CodeNoise = CPNoise = ElevThreshold = 0.0;
		RatioThres = 3.0;

		// 初始化字符串成员为空
		memset(BasNetIP, 0, sizeof(BasNetIP));
		memset(RovNetIP, 0, sizeof(RovNetIP));
		memset(BasObsDatFile, 0, sizeof(BasObsDatFile));
		memset(RovObsDatFile, 0, sizeof(RovObsDatFile));
		memset(ResFile, 0, sizeof(ResFile));
		memset(DiffFile, 0, sizeof(DiffFile));
	}

};

bool ReadRTKConfigInfo(const char FName[], ROVERCFGINFO& ANInfo);

template <typename Derived>//保存浮点解过程矩阵P、w、B
void SaveEigen(const Eigen::MatrixBase<Derived>& mat, const std::string& filename)//C++ 模板函数的定义通常需要与声明放在同一个头文件中,模板函数需要在编译时根据具体类型进行实例化
{
	std::ofstream fout(filename);
	if (fout.is_open()) {
		fout << mat << std::endl;
		fout.close();
	}
	else {
		std::cerr << "Cannot open file: " << filename << std::endl;
	}
}

//声明通用时,简化儒略日和GPS时之间的相互转换函数
void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT);
void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT);
void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT);
void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT);
void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT);
void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT);

//空间直角坐标,大地坐标的相互转换函数
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F);
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F);
void BLHToNEUMat(const double Blh[], double Mat[3][3]);
void CompSatElAz(const double Xr[], const double Xs[], const double R, const double F, double* Elev, double* Azim); //卫星高度角方位角计算函数
void Comp_dEnu(const double X0[], const double Xr[], const double R, const double F, double dNeu[]);  //定位误差计算函数

// NovAtel OEM7数据解码函数：要换成eigen库
short S2(char* p);
unsigned short US2(unsigned char* p);
int I4(char* p);
unsigned int UI4(unsigned char* p);
float F4(unsigned char* p);
double D8(unsigned char* p);
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);
int decode_rangeb_oem7(unsigned char* Buff,  EPOCHOBSDATA* obs);
int decode_gpsephem(unsigned char* buff, GPSEPHREC* geph);
int decode_bdsephem(unsigned char* buff, GPSEPHREC* beph);
int decode_psrpos(unsigned char* buff, POSRES* pos);
unsigned int crc32(const unsigned char* buff, int len);

// 卫星位置
void SatellitePosition(GPSEPHREC geph[], GPSEPHREC beph[], EPOCHOBSDATA* obs, double UserPos[3]);
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATMIDRES* Mid);
bool CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);
bool CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATMIDRES* Mid);
void RotationCorrection(SATMIDRES* Mid, double UserPos[3], double Omega_e);//对卫星位置进行地球自转改正

//误差改正
void DetectOutlier(EPOCHOBSDATA* obs); 
double hopfield(double hgt, double elev);

//SPP & SPV
bool SPP(EPOCHOBSDATA* Epoch, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, POSRES* Res);
bool SPV(EPOCHOBSDATA* obs, POSRES* Res);
bool SPP_LS(EPOCHOBSDATA* obs, POSRES* Res);

//RTK_LS
int TimeSyn(FILE* fbase, FILE* frover, RAWDAT* Raw);
int TimeSynSoc(SOCKET& BasSoc, SOCKET& RovSoc, RAWDAT* Raw);
void MarkValid(const EPOCHOBSDATA* Epk0, EPOCHOBSDATA* Epk);
void FormSDEpochObs(const EPOCHOBSDATA* RovEpk, const EPOCHOBSDATA* BasEpk, SDEPOCHOBS* SDObs);
void DetectCycleSlip(SDEPOCHOBS* SDObs);
void DetRefSat(const EPOCHOBSDATA* RovEpk, const EPOCHOBSDATA* BasEpk, SDEPOCHOBS* SDObs, DDCOBS* DDObs);
bool RTKFloat(RAWDAT* Raw, POSRES* Base, POSRES* Rover, FloatResult* Fres);
bool RTKFix(RAWDAT* Raw, POSRES* Base, POSRES* Rove);

//RTK_EKF
bool InitFilter(RAWDAT* Raw, POSRES* Base, POSRES* Rov, RTKEKF* ekf);
bool EkfPredict(RAWDAT* Raw, POSRES* Rov, RTKEKF* ekf);
bool EkfMeasureUpdate(RAWDAT* Raw, RTKEKF* ekf);

//lambda
void CopyArray(int count, double dest[], const double src[]);
int MatrixInv(int n, double a[], double b[]);
int MatrixInv_SRS(const int n, double a[]);
void MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);
int LD(int n, const double* Q, double* L, double* D);
void gauss(int n, double* L, double* Z, int i, int j);
void perm(int n, double* L, double* D, int j, double del, double* Z);
void reduction(int n, double* L, double* D, double* Z);
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);