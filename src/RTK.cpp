#include "SPP.h"
#include"Socket.h"
extern ROVERCFGINFO cfginfo;

/****************************************************************************
数据同步函数：静态场景下小于30s，动态场景要小于一个微小量

目的：文件模式下进行基站、流动的数据解码与获取，并进行时间同步

输入参数：
      FBas 指向基站数据文件的指针
      FRov 指向流动站数据文件的指针
      Raw  同步数据存储处（两个站的观测值、星历、参考定位结果）
输出参数：数据同步结果的标志  1-数据同步成功  0-数据同步失败 -1-文件数据结束
*****************************************************************************/
int TimeSyn(FILE* fbase, FILE* frover, RAWDAT* Raw) //注意传文件指针
{
    // 将上一历元的两站数据存储好
    memcpy(&Raw->BasEpk0, &Raw->BasEpk, sizeof(EPOCHOBSDATA));
    memcpy(&Raw->RovEpk0, &Raw->RovEpk, sizeof(EPOCHOBSDATA));


    //解码本历元流动站、基站的数据，进行时间同步
    static unsigned char RBuff[MAXRAWLEN], BBuff[MAXRAWLEN]; //!要注意生命周期 static ,=0 的初始化操作只在函数第一次被调用时执行一次
    int RLenR, BLenR;         //已接收数据长度，实际接收长度
    static int RLenD = 0, BLenD = 0; //剩余长度，要注意生命周期 static ,=0 的初始化操作只在函数第一次被调用时执行一次

    // 1.获取流动站的观测数据，若不成功，继续获取，若文件结束，返回;若成功，得到观测时刻。
    while (!feof(frover))
    {
        if ((RLenR = fread(RBuff + RLenD, sizeof(unsigned char), MAXRAWLEN - RLenD, frover)) < MAXRAWLEN - RLenD)  return -1;//为什么用MAXRAWLEN - RLenD判定，而不是1，都ok吧
        RLenD += RLenR;
        if (DecodeNovOem7Dat(RBuff, RLenD, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->bestpos_rover) == 1) break;
    }

    // 2.流动站观测时刻与当前基站观测时刻比较，在限差范围内，文件同步成功，返回1；
    double dt = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);
    if (fabs(dt) < 0.01) { return 1; }
    else
    {
        // 3.若不在限差范围内，获取基站观测数据，两站的观测时间求差，在限差范围内，同步成功返回1；
        // 4.如果不在限差范围内，若基站时间在后，返回0；若基站时间在前，循环获取基站数据，直到成功。
        while (!feof(fbase))
        {
            if ((BLenR = fread(BBuff + BLenD, sizeof(unsigned char), MAXRAWLEN - BLenD, fbase)) < MAXRAWLEN - BLenD) return -1;
            BLenD += BLenR;
            if (DecodeNovOem7Dat(BBuff, BLenD, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->bestpos_base) == 1)            //之前的卫星星历数据被覆盖
            {
                dt = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);
                if (fabs(dt) < 0.01) { return 1; }
                else if (dt < 0.5) { return 0; } // 流动站数据滞后，无法解算！
                else; // dt > 0 时继续读取基站数据（基站时间在前）
            }
        }
    }

}


/****************************************************************************
数据同步函数：静态场景下小于30s，动态场景要小于一个微小量

目的：网络模式下进行基站、流动的数据解码与获取，并进行时间同步

输入参数：
      BasSock 基站数据的网口
      RovSock 流动站数据的网口
      Raw  同步数据存储处（两个站的观测值、星历、参考定位结果）
输出参数：数据同步结果的标志  1-数据同步成功  0-数据同步失败 //-1-文件数据结束
*****************************************************************************/
int TimeSynSoc(SOCKET& BasSoc, SOCKET& RovSoc, RAWDAT* Raw)
{
    // 将上一历元的两站数据存储好
    memcpy(&Raw->BasEpk0, &Raw->BasEpk, sizeof(EPOCHOBSDATA));
    memcpy(&Raw->RovEpk0, &Raw->RovEpk, sizeof(EPOCHOBSDATA));

    Sleep(980);

    //解码本历元流动站、基站的数据，进行时间同步
    static unsigned char RBuff[MAXRAWLEN], BBuff[MAXRAWLEN]; //!要注意生命周期 static ,=0 的初始化操作只在函数第一次被调用时执行一次
    int RLenR, BLenR;         //已接收数据长度，实际接收长度
    static int RLenD = 0, BLenD = 0; //剩余长度，要注意生命周期 static ,=0 的初始化操作只在函数第一次被调用时执行一次

    //流动站
    if ((RLenR = recv(RovSoc, (char*)RBuff, 40960 - RLenD, 0)) > 0)
    {
        RLenD += RLenR;
        DecodeNovOem7Dat(RBuff, RLenD, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph, &Raw->bestpos_rover);
    }

    //基站
    if ((BLenR = recv(BasSoc, (char*)BBuff, 40960 - BLenD, 0)) > 0)
    {
        BLenD += BLenR;
        DecodeNovOem7Dat(BBuff, BLenD, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->bestpos_base);
    }

    //求时间差
    double dt = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);
    if (fabs(dt) < 5 ) { return 1; }
    else
    {
        //cout << dt << " ";
        return 0;
    }

    //else          //大于限差才更新容易出现一些浮点解的情况
    //{
    //    if ((BLenR = recv(BasSoc, (char*)BBuff, 40960 - BLenD, 0)) > 0)
    //    {
    //        BLenD += BLenR;
    //        DecodeNovOem7Dat(BBuff, BLenD, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph, &Raw->bestpos_base);
    //    }
    //    dt = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800 + (Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek);
    //    if (fabs(dt) < 5) { return 1; }
    //    else
    //    {

    //        cout << dt << "  ";
    //        return 0;
    //    }

    //}
}


/*******************************************************************
valid有效性标记函数

目的：根据locktime进行非差观测数据有效性标记

参数：Epk0 指向上一历元观测数据的指针
      Epk 指向当前历元观测数据的指针
*******************************************************************/
void MarkValid(const EPOCHOBSDATA* Epk0, EPOCHOBSDATA* Epk)
{
    //Locktime：判断历元间相位数据的连续性，没有发生中断时Locktime(k + 1) >= Locktime(k)，数据正常；反之有周跳将观测结构体中的Valid设置为false
    int i, j, n;

    //遍历本历元观测数据中的每一颗卫星
    for (i = 0; i < Epk->SatNum; i++)
    {
        //寻找该卫星在上一历元中的数据
        for (j = 0; j < Epk0->SatNum; j++)
        {
            if (Epk0->SatObs[j].System == Epk->SatObs[i].System && Epk0->SatObs[j].Prn == Epk->SatObs[i].Prn)
            {
                //对每个频率进行检测，不要求必须为双频数据
                for (n = 0; n < 2; n++)
                {
                   
                        //判断历元间相位数据的连续性，没有发生中断时Locktime(k + 1) >= Locktime(k)，数据正常；反之有周跳
                        if (Epk->SatObs[i].LockTime[n] < Epk0->SatObs[j].LockTime[n]) Epk->SatObs[i].Valid = false;
                }
            }
            else continue;
        }
    }

}

/***************************************************************
站间单差

目的：将流动站与基站的观测数据求差，，并标记是否可能存在半周

参数：RovEpk  指向流动站观测数据的指针
      BasEpk  指向基站观测数据的指针
      SDObs   单差结果存储处
***************************************************************/
void FormSDEpochObs(const EPOCHOBSDATA* RovEpk, const EPOCHOBSDATA* BasEpk, SDEPOCHOBS* SDObs)
{
    int satnum_sd = 0;
    int i, j;//循环流动站和基站卫星
    int k;//循环两个频率

    //根据流动站卫星数进行循环，对于每颗流动站的卫星:
    for (i = 0; i < RovEpk->SatNum; i++)
    {
        //    1. 检查卫星号和系统号是否正常，卫星观测值是否完整（两个频率的伪距和载波相位都存在），如果不正常，continue；
        if (RovEpk->SatObs[i].Valid == false || RovEpk->SatPVT[i].Valid == false) continue;

        //    2. 在基站观测值中查找与该流动站相同的卫星，如果未找到，continue；
        for (j = 0; j < BasEpk->SatNum; j++)
        {
            if (BasEpk->SatObs[j].Valid == false || BasEpk->SatPVT[j].Valid == false) continue;

            if (BasEpk->SatObs[j].System == RovEpk->SatObs[i].System && BasEpk->SatObs[j].Prn == RovEpk->SatObs[i].Prn)
            {
                //    3. 对同类型和同频率的观测值求差并保存，保存索引号、卫星号和系统号等相关信息，累加单差观测值卫星数量
                SDObs->SdSatObs[satnum_sd].System = RovEpk->SatObs[i].System;
                SDObs->SdSatObs[satnum_sd].Prn = RovEpk->SatObs[i].Prn;
                SDObs->SdSatObs[satnum_sd].nRov = i;
                SDObs->SdSatObs[satnum_sd].nBas = j;

                for (k = 0; k < 2; k++)
                {
                    SDObs->SdSatObs[satnum_sd].dP[k] = RovEpk->SatObs[i].P[k] - BasEpk->SatObs[j].P[k];
                    SDObs->SdSatObs[satnum_sd].dL[k] = RovEpk->SatObs[i].L[k] - BasEpk->SatObs[j].L[k];
                    if (RovEpk->SatObs[i].half[k] == 1 && BasEpk->SatObs[j].half[k] == 1) SDObs->SdSatObs[satnum_sd].half[k] = 1; // 不存在半周问题
                    else SDObs->SdSatObs[satnum_sd].half[k] = 0; // 可能存在半周问题
                }
                satnum_sd++;
                break;
            }
            else continue;
        }
    }

    //    4. 循环结束后，对单差观测值的观测时刻和卫星数量赋值。
    SDObs->Time = RovEpk->Time;
    SDObs->SatNum = satnum_sd;

}

/***************************************************************
周跳探测函数

目的：对每个卫星的单差观测数据进行有效性标记，计算每个历元MW、GF组合观测值并用其来进行周跳探测

参数：SDObs  指向每个历元的单差观测数据的指针

***************************************************************/
void DetectCycleSlip(SDEPOCHOBS* SDObs)
{
    //定义MWGF SDComObs[MAXCHANNUM]，用于临时存放当前历元的计算结果
    MWGF SDComObs[MAXCHANNUM];
    int i, j;
    double f[2];
    double dGF = 0.0 ,dMW = 0.0;

    //对于任意一颗卫星的单差观测数据，其周跳探测的步骤为
    for (i = 0; i < SDObs->SatNum; i++)
    {
        //    1. 检查该卫星的单差伪距和相位数据是否有效和完整（不要求必须为双频数据），若为0，将Valid标记为false，continue
        if (fabs(SDObs->SdSatObs[i].dL[0]) < 1e-8 || fabs(SDObs->SdSatObs[i].dL[1]) < 1e-8 || fabs(SDObs->SdSatObs[i].dP[0]) < 1e-8 || fabs(SDObs->SdSatObs[i].dP[1]) < 1e-8)
        {
            SDObs->SdSatObs[i].Valid = false;
            continue;//跳过无效数据
        }
        else;

        //    2. 计算当前历元该卫星的GF和MW组合值
        SDComObs[i].Sys = SDObs->SdSatObs[i].System;
        SDComObs[i].Prn = SDObs->SdSatObs[i].Prn;
        if (SDComObs[i].Sys == GPS)
        {
            f[0] = FG1_GPS;
            f[1] = FG2_GPS;
        }
        else if (SDComObs[i].Sys == BDS)
        {
            f[0] = FG1_BDS;
            f[1] = FG3_BDS;
        }
        else continue;
        SDComObs[i].GF = SDObs->SdSatObs[i].dL[0] - SDObs->SdSatObs[i].dL[1];
        SDComObs[i].MW= 1 / (f[0] - f[1]) * (f[0] * SDObs->SdSatObs[i].dL[0] - f[1] * SDObs->SdSatObs[i].dL[1])
            - 1 / (f[0] + f[1]) * (f[0] * SDObs->SdSatObs[i].dP[0] + f[1] * SDObs->SdSatObs[i].dP[1]);

        //    3. 从上个历元的MWGF数据中查找该卫星的GF和MW组合值
        for (j = 0; j < MAXCHANNUM; j++)
        {
            if (SDObs->SdCObs[j].Sys == SDComObs[i].Sys && SDObs->SdCObs[j].Prn == SDComObs[i].Prn)
            {
                //    4. 计算当前历元该卫星GF与上一历元对应GF的差值dGF
                dGF = SDComObs[i].GF - SDObs->SdCObs[j].GF;
                //    5. 计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
                dMW = SDComObs[i].MW - SDObs->SdCObs[j].MW;

                //    6. 检查dGF和dMW是否超限，限差阈值建议为5cm和3m。若超限，标记为粗差，将Valid标记为
                //    false ，若不超限，标记为可用将Valid标记为true，并计算该卫星的MW平滑值
                if (fabs(dGF) < 0.05 && fabs(dMW) < 3) 
                {
                    SDObs->SdSatObs[i].Valid = true;
                    // 计算 MW 平滑值
                    SDComObs[i].n = SDObs->SdCObs[j].n + 1;
                    SDComObs[i].MW = (SDObs->SdCObs[j].MW * SDObs->SdCObs[j].n + SDComObs[i].MW) / SDComObs[i].n;
                }
                else
                {
                    SDObs->SdSatObs[i].Valid = false;
                    SDComObs[i].n = 1;
                }

                break;

            }
            else continue;
        }

    }

    //    7. 所有卫星循环计算完成之后，将SDComObs内存拷贝到SDObs->SdCObs，即函数运行结束后，SDObs->SdCObs保存了当前历元的GF和MW平滑值。
    memcpy(SDObs->SdCObs, SDComObs, sizeof(MWGF)* MAXCHANNUM);//sizeof中最好放结构体的大小

}


/**************************************************************************************
基准星选取函数

目的：选取基准星（双系统各一个），将其PRN号与索引存放在双差数据中

参数：RovEpk   指向流动站观测数据的指针
      BasEpk   指向基站观测数据的指针
      SDObs    指向单差数据的指针
      DDObs    指向双差数据的指针

判断是否写成功：未成功选取该历元下的基准星，则DDObs->RefPos[j] = -1;
**************************************************************************************/
void DetRefSat(const EPOCHOBSDATA* RovEpk, const EPOCHOBSDATA* BasEpk, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
    //每个卫星系统各选取一颗卫星作为参考星，0 = GPS; 1 = BDS
    short Sys = 0;  //0 = GPS; 1 = BDS
    int i, j;
    double CN_Ele[2] = { 0.0 }, MaxCN_Ele[2] = { 0.0 };//两个频率CN0与高度角之和
    int RefPrn[2] = { -1 }, RefIndex[2] = { -1 };

    //遍历单差数据
    for (i = 0; i < SDObs->SatNum; i++)
    {
        //    1. 伪距和载波相位通过周跳探测，没有粗差和周跳以及半周标记，再斟酌一下
        if (SDObs->SdSatObs[i].Valid == false || SDObs->SdSatObs[i].half[0] == 0 || SDObs->SdSatObs[i].half[1] == 0) continue;
        //    2. 卫星星历正常，卫星位置计算成功
        else if (RovEpk->SatPVT[SDObs->SdSatObs[i].nRov].Valid == false || BasEpk->SatPVT[SDObs->SdSatObs[i].nBas].Valid == false) continue;
        //    3. 连续观测时间大于一定时间，6s
        else if (RovEpk->SatObs[SDObs->SdSatObs[i].nRov].LockTime[0] < 6 || RovEpk->SatObs[SDObs->SdSatObs[i].nRov].LockTime[1] < 6 || BasEpk->SatObs[SDObs->SdSatObs[i].nBas].LockTime[0] < 6 || BasEpk->SatObs[SDObs->SdSatObs[i].nBas].LockTime[1] < 6) continue;
        else;

        Sys = (SDObs->SdSatObs[i].System == GPS) ? 0 : 1;   //0 = GPS; 1 = BDS
        CN_Ele[Sys] = RovEpk->SatPVT[SDObs->SdSatObs[i].nRov].Elevation + RovEpk->SatObs[SDObs->SdSatObs[i].nRov].cn[0] + RovEpk->SatObs[SDObs->SdSatObs[i].nRov].cn[1]
            + BasEpk->SatObs[SDObs->SdSatObs[i].nBas].cn[0] + BasEpk->SatObs[SDObs->SdSatObs[i].nBas].cn[1];
        
        //    4. 卫星高度角大(计算存储在SatPVT中）或者cn0大（解码在SatObs中，有两个频率）
        if (CN_Ele[Sys] > MaxCN_Ele[Sys])
        {
            MaxCN_Ele[Sys] = CN_Ele[Sys]; 
            RefPrn[Sys] = SDObs->SdSatObs[i].Prn;//加个卫星号调试一下
            RefIndex[Sys] = i;
        }
    }

    for (j = 0; j < 2; j++)
    {
        if (MaxCN_Ele[j] < 160.0) DDObs->RefPos[j] = -1;
        else
        {
            DDObs->RefPrn[j] = RefPrn[j];
            DDObs->RefPos[j] = RefIndex[j];
        }

    }
}

/******************************************************************************************
获取最小二乘相对定位浮点解

目的：进行浮点解计算，保存双差浮点解模糊度及其协因数矩阵到Fres

输入参数：
      Raw    指向历元总数据的指针
      Base   基站的SPP结果线性化用解码得到的准确坐标
      Rover    流动站的SPP结果（用于初始化坐标）
输出参数：
      Fres   浮点解结果
返回值：浮点解成功则返回true，失败则返回false

容错处理：
    SPP是否成功               //注意
    参考星选取是否成功        //注意
    双差观测值数量是否足够
    矩阵求逆是否成功

注意：在开始定义变量在整个函数中都可见，将一些变量初始化，养成习惯
******************************************************************************************/
bool RTKFloat(RAWDAT* Raw, POSRES* Base, POSRES* Rover, FloatResult* Fres)
{
    if (Base->IsSuccess == false || Rover->IsSuccess == false)
    {
        cout << "SPP解算失败不进行相对定位浮点解计算" << endl;
        return false;
    }
    if(Raw->DDObs.RefPos[0]==-1|| Raw->DDObs.RefPos[1] == -1)
    {
        cout << "参考星选取失败不进行相对定位浮点解计算" << endl;
        return false;
    }
    
    //    1. 设置基站和流动站位置初值（基站为解码得到的值，流动站为SPP得到的值）
    double BasPos[3] = { Raw->bestpos_base.Pos[0],Raw->bestpos_base.Pos[1],Raw->bestpos_base.Pos[2] };
    double RovPos[3] = { Rover->Pos[0],Rover->Pos[1], Rover->Pos[2] };

    //    2. 计算GPS和BDS双差卫星数（该程序为双系统双频；若为单双频混用：各系统每个频率的双差卫星数）
    short SatNum=0, GPS_SatNum=0, BDS_SatNum=0;
    if (Raw->SdObs.SatNum < 2) return false;//单差观测值小于2，无法进行双差
    for (int m = 0; m < Raw->SdObs.SatNum; m++)
    {
        if (m == Raw->DDObs.RefPos[0] || m == Raw->DDObs.RefPos[1]) continue; // 基准星不计入可用双差卫星数
        if (Raw->SdObs.SdSatObs[m].Valid==false||Raw->SdObs.SdSatObs[m].half[0]==0|| Raw->SdObs.SdSatObs[m].half[1] == 0)continue;
        
        if (Raw->SdObs.SdSatObs[m].System == GPS) GPS_SatNum++;
        else BDS_SatNum++;
    }
    SatNum = GPS_SatNum + BDS_SatNum;

    //根据双差卫星数定义矩阵变量
    int row = 4 * (GPS_SatNum + BDS_SatNum);       //行，方程个数
    int col = 3 + 2 * (GPS_SatNum + BDS_SatNum);   //列，未知数个数
    double BasToSats[MAXCHANNUM];// 基站坐标到所有卫星的几何距离（下标与基站原始观测数据的索引一致），大小可以设置为Raw->BasEpk.SatNum还是MAXCHANNUM？
    double RovToRefSat[2] = { 0,0 };// 2个系统流动站到参考星的距离
    Eigen::VectorXd DDN = Eigen::VectorXd::Zero(col-3); // 双差模糊度
    Eigen::VectorXd x = Eigen::VectorXd::Zero(col);          // 待估参数改正数
    Eigen::VectorXd w = Eigen::VectorXd::Zero(row);          // 
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(row, col); // 设计矩阵
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(row, row); // 权阵
    Eigen::MatrixXd Q_xx = Eigen::MatrixXd::Zero(col, col); // 协因数阵
    Eigen::VectorXd V = Eigen::VectorXd::Zero(row);          // V=Bx-w，计算残差平方和rms

    //    3. 计算基站坐标到所有卫星的几何距离，存储到数组中；得到参考卫星位置
    for (int n = 0; n < Raw->BasEpk.SatNum; n++)
    {
        BasToSats[n] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[n].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[n].SatPos[0])
            + (BasPos[1] - Raw->BasEpk.SatPVT[n].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[n].SatPos[1])
            + (BasPos[2] - Raw->BasEpk.SatPVT[n].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[n].SatPos[2]));
    }
    // 提取参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
    int RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    int RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // 参考卫星的位置（对于流动站和基站来说，由于时间不一定完全同步,故各自用各自星历所得的卫星位置）
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };

    
    //最小二乘迭代控制
    const int maxIterations = 15;
    const double convergenceThreshold = 1e-8;
    int iteration = 0;
    bool converged = false;
    double deltaNorm;

    while (!converged && iteration < maxIterations)
    {
        //迭代之前将向量和矩阵重置为0
        x.setZero();
        w.setZero();
        B.setZero();
        P.setZero();
        Q_xx.setZero();
        
        //    4. 计算流动站到参考星的几何距离 0=GPS, 1=BDS
        RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
        RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));
        
        //    5. 对单差观测值进行循环
        int index = 0;    //记录可用的双差观测
        for (int i = 0; i < Raw->SdObs.SatNum; i++)
        {
            //    ① 参考星不用计算，存在半周不参与计算
            if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue;
            else if (Raw->SdObs.SdSatObs[i].Valid == false || Raw->SdObs.SdSatObs[i].half[0] == 0 || Raw->SdObs.SdSatObs[i].half[1] == 0)continue;
            else;
        
            //    ② 线性化双差观测方程得到B矩阵和W向量
            
            // 记录当前卫星在流动站原始观测数据中的索引
            short satindex = Raw->SdObs.SdSatObs[i].nRov;
            // 确定本颗卫星所属系统
            int sys = 0;
            sys = (Raw->SdObs.SdSatObs[i].System == GPS) ? 0 : 1;   //0 = GPS; 1 = BDS
            // 计算流动站到本颗卫星的几何距离
            double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));

            //B矩阵
            double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
            double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
            double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
            for (int j = 0; j < 4; j++)
            {
                B(index * 4 + j, 0) = l;
                B(index * 4 + j, 1) = m;
                B(index * 4 + j, 2) = n;
                if (j == 2) B(index * 4 + j, 3 + index * 2) = (sys == 0) ? WL1_GPS : WL1_BDS;
                if (j == 3) B(index * 4 + j, 3 + index * 2 + 1) = (sys == 0) ? WL2_GPS : WL3_BDS;
            }

            //计算双差观测值和双差模糊度
            double DDP[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double DDL[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            double DDrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];//利用前面定义的参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
            DDN(index * 2) = (DDL[0] - DDP[0]) / ((sys == 0) ? WL1_GPS : WL1_BDS);
            DDN(index * 2 + 1) = (DDL[1] - DDP[1]) / ((sys == 0) ? WL2_GPS : WL3_BDS);
            //W向量
            w(index * 4) = DDP[0] - DDrou;
            w(index * 4 + 1) = DDP[1] - DDrou;
            w(index * 4 + 2) = DDL[0] - DDrou - DDN(index * 2) * ((sys == 0) ? WL1_GPS : WL1_BDS);
            w(index * 4 + 3) = DDL[1] - DDrou - DDN(index * 2 + 1) * ((sys == 0) ? WL2_GPS : WL3_BDS);
          
            index++;
        }
        //可以输出看一下SatNum和index的大小是不是相等的，按理来说应该是相等的

        //    ③ 计算权矩阵, GPS、BDS对应的P、L填充值，注意1.0将除法转换为保留小数的运算

        //采用经验噪声设置权阵的大小
        //double GP[4] = { GPS_SatNum / (GPS_SatNum + 1.0) / 2 / 0.09, GPS_SatNum / (GPS_SatNum + 1.0) / 2 / 0.0001, -1 / (GPS_SatNum + 1.0) / 2 / 0.09, -1 / (GPS_SatNum + 1.0) / 2 / 0.0001 };
        //double BD[4] = { BDS_SatNum / (BDS_SatNum + 1.0) / 2 / 0.09, BDS_SatNum / (BDS_SatNum + 1.0) / 2 / 0.0001, -1 / (BDS_SatNum + 1.0) / 2 / 0.09, -1 / (BDS_SatNum + 1.0) / 2 / 0.0001 };

        //从配置文件读取
        double GP[4] = { GPS_SatNum / (GPS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise , GPS_SatNum / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise, -1 / (GPS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise, -1 / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };
        double BD[4] = { BDS_SatNum / (BDS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise , BDS_SatNum / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise, -1 / (BDS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise, -1 / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };


        //填充GPS
        for (int a = 0;a < 4 * GPS_SatNum; a++)//循环行
        {
            for (int b = 0; b < 4 * GPS_SatNum; b++)//循环列
            {
                if (b == a)//对角线上
                {
                    if (a % 4 == 0 || a % 4 == 1) P(a, b) = GP[0];
                    else P(a, b) = GP[1];
                }
                else if (b % 4 == a % 4)//非对角线上
                {
                    if(a % 4==0|| a % 4 == 1)  P(a, b) = GP[2];
                    else P(a, b) = GP[3];
                }
            }
        }
        //填充BDS
        for (int a = 0; a < 4 * BDS_SatNum; a++)//循环行
        {
            for (int b = 0; b < 4 * BDS_SatNum; b++)//循环列
            {
                if (b == a)//对角线上
                {
                    if (a % 4 == 0 || a % 4 == 1) P(a + 4 * GPS_SatNum, b+ 4 * GPS_SatNum) = BD[0];
                    else  P(a + 4 * GPS_SatNum, b + 4 * GPS_SatNum) = BD[1];
                }
                else if (b % 4 == a % 4)//非对角线上
                {
                    if (a % 4 == 0 || a % 4 == 1)  P(a + 4 * GPS_SatNum, b + 4 * GPS_SatNum) = BD[2];
                    else P(a + 4 * GPS_SatNum, b + 4 * GPS_SatNum) = BD[3];
                }
            }
        }
        
        //    6. 双差观测方程数量大于未知数，则求解
        if (SatNum < 5) return false; // 观测不足以解算
        else;

        //    7. 最小二乘解算
        MatrixXd N = B.transpose() * P * B;
        VectorXd L = B.transpose() * P * w;
        x = N.inverse() * L;
        Q_xx = N.inverse();

        //    8. 更新流动站的位置和双差模糊度参数
        RovPos[0] += x(0);
        RovPos[1] += x(1);
        RovPos[2] += x(2);
        for (int i = 0; i < SatNum * 2; i++)
        {
            DDN(i) += x(i + 3);
        }

        //    9. 流动站位置增量大于阈值或迭代次数小于阈值，返回4迭代计算，否则浮点解计算完成。
        deltaNorm = sqrt(fabs(x(0)) * fabs(x(0)) + fabs(x(1)) * fabs(x(1)) + fabs(x(2)) * fabs(x(2)));
        converged = (deltaNorm < convergenceThreshold);
        iteration++;

    }
    
    //    10. 精度评价
    V = B * x - w;
    double sigma = sqrt((V.transpose() * P * V)(0) / (2 * SatNum - 3));

    //    11. 保存双差浮点解模糊度及其协因数矩阵，用于LAMBDA模糊度固定
    //解算所用双差卫星数
    Raw->DDObs.DDSatNum[0] = Fres->stanum[0] = GPS_SatNum;
    Raw->DDObs.DDSatNum[1] = Fres->stanum[1] = BDS_SatNum;
    Raw->DDObs.Sats = Fres->totalsatnum = SatNum;
    //浮点解基线向量，用于检验
    Raw->DDObs.dPos_float[0] = Fres->dX[0] = RovPos[0] - BasPos[0];
    Raw->DDObs.dPos_float[1] = Fres->dX[1] = RovPos[1] - BasPos[1];
    Raw->DDObs.dPos_float[2] = Fres->dX[2] = RovPos[2] - BasPos[2];
    //双差模糊度
    memset(Fres->N, 0, sizeof(Fres->N));//避免上一个历元双差模糊度个数的影响
    for (int i = 0; i < SatNum * 2; i++)
    {
        Fres->N[i] = DDN(i);
    }
    //单位权误差
    Fres->sigma = sigma;
    //DOP、PDOP
    Raw->DDObs.PDOP = sqrt(Q_xx(0, 0) + Q_xx(1, 1) + Q_xx(2, 2));
    
    //模糊度协因数矩阵
    int amb_dim = 2 * SatNum;  // 模糊度参数维数
    Eigen::MatrixXd Q_amb = Q_xx.block(3, 3, amb_dim, amb_dim);// 取出模糊度协因数子矩阵（右下角部分）
    memset(Fres->Q, 0, sizeof(Fres->Q));
    for (int i = 0; i < amb_dim; i++) // Eigen矩阵是按列优先存储，数组按行优先访问
    {
        for (int j = 0; j < amb_dim; j++) 
        {
            Fres->Q[i * amb_dim + j] = Q_amb(i, j);
        }
    }
    
    // 保存 B, w, P, x 到文件 
    //SaveEigen(B, "B.txt");        // 保存设计矩阵 B (row x col)
    //SaveEigen(w, "w.txt");        // 保存残差向量 w (row x 1)
    //SaveEigen(P, "P.txt");        // 保存权阵 P (row x row)
    //SaveEigen(x, "x.txt");        // 保存参数改正向量 x (col x 1)
    //SaveEigen(Q_amb, "Qnn.txt");


    return true;
}

/******************************************************************************************
获取最小二乘相对定位固定解（固定解N带入载波相位观测方程）

目的：进行固定解计算，将固定解N带入载波相位观测方程得到精确定位，并存储结果至Raw中
      m个GPS卫星，n个BDS卫星，载波相位观测方程2*（m-1)+2*(n-1)，未知数个数3

输入参数：
      Raw    指向历元总数据的指针
      Base   基站的SPP结果线性化用解码得到的准确坐标
      Rov    流动站的SPP结果（用于初始化坐标）
输出参数：
      Raw->DDObs
返回值：固定解解成功则返回true，失败则返回false

容错处理：在浮点解部分已经进行了容错处理
******************************************************************************************/
bool RTKFix(RAWDAT* Raw, POSRES* Base, POSRES* Rover)
{
    //    1. 设置基站和流动站位置初值
    double BasPos[3] = { Raw->bestpos_base.Pos[0],Raw->bestpos_base.Pos[1],Raw->bestpos_base.Pos[2] };
    double RovPos[3] = { Rover->Pos[0],Rover->Pos[1], Rover->Pos[2] };
    
    //    2. 调用Raw->DDObs中GPS和BDS双差卫星数
    short SatNum = Raw->DDObs.Sats, GPS_SatNum = Raw->DDObs.DDSatNum[0], BDS_SatNum = Raw->DDObs.DDSatNum[1];

    //根据双差卫星数定义矩阵变量
    int row = 2 * (GPS_SatNum + BDS_SatNum);       //行，方程个数
    int col = 3;   //列，未知数个数
    double BasToSats[MAXCHANNUM];// 基站坐标到所有卫星的几何距离（下标与基站原始观测数据的索引一致），大小可以设置为Raw->BasEpk.SatNum还是MAXCHANNUM？
    double RovToRefSat[2] = { 0,0 };// 流动站到2个系统参考星的距离
    Eigen::VectorXd DDN = Eigen::VectorXd::Zero(2 * (GPS_SatNum + BDS_SatNum)); // 双差模糊度
    Eigen::VectorXd x = Eigen::VectorXd::Zero(col);          // 待估参数改正数
    Eigen::VectorXd w = Eigen::VectorXd::Zero(row);          // 
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(row, col); // 设计矩阵
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(row, row); // 权阵
    Eigen::MatrixXd Q_xx = Eigen::MatrixXd::Zero(col, col); // 协因数阵
    //Eigen::VectorXd V = Eigen::VectorXd::Zero(row);          // V=Bx-w，计算残差平方和rms

    //lambda固定解N给DNN赋值
    for (int a = 0; a < 2 * (GPS_SatNum + BDS_SatNum); a++)
    {
        DDN(a) = Raw->DDObs.FixedAmb[a];
    }

    //    3. 计算基站坐标到所有卫星的几何距离
    for (int n = 0; n < Raw->BasEpk.SatNum; n++)
    {
        BasToSats[n] = sqrt((BasPos[0] - Raw->BasEpk.SatPVT[n].SatPos[0]) * (BasPos[0] - Raw->BasEpk.SatPVT[n].SatPos[0])
            + (BasPos[1] - Raw->BasEpk.SatPVT[n].SatPos[1]) * (BasPos[1] - Raw->BasEpk.SatPVT[n].SatPos[1])
            + (BasPos[2] - Raw->BasEpk.SatPVT[n].SatPos[2]) * (BasPos[2] - Raw->BasEpk.SatPVT[n].SatPos[2]));
    }
    // 提取参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
    int RefSatOfRov[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nRov,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nRov };
    int RefSatOfBas[2] = { Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[0]].nBas,Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[1]].nBas };
    // 参考卫星的位置（对于流动站和基站来说，由于时间不一定完全同步,故各自用各自星历所得的卫星位置）
    double RefSatPosOfGPSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[0]].SatPos[2] };
    double RefSatPosOfBDSR[3] = { Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[0],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[1],Raw->RovEpk.SatPVT[RefSatOfRov[1]].SatPos[2] };

    //最小二乘迭代控制
    const int maxIterations = 15;
    const double convergenceThreshold = 1e-8;
    int iteration = 0;
    bool converged = false;
    double deltaNorm;

    while (!converged && iteration < maxIterations)
    {
        //迭代之前将向量和矩阵重置为0
        x.setZero();
        w.setZero();
        B.setZero();
        P.setZero();
        Q_xx.setZero();

        //    4. 计算流动站到参考星的几何距离 0=GPS, 1=BDS
        RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
        RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));

        //    5. 对单差观测值进行循环
        int index = 0;    //记录可用的双差观测
        for (int i = 0; i < Raw->SdObs.SatNum; i++)
        {
            //    ① 参考星不用计算，存在半周不参与计算
            if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue;
            else if (Raw->SdObs.SdSatObs[i].Valid == false || Raw->SdObs.SdSatObs[i].half[0] == 0 || Raw->SdObs.SdSatObs[i].half[1] == 0)continue;
            else;

            //    ② 线性化双差观测方程得到B矩阵和W向量

            // 记录当前卫星在流动站原始观测数据中的索引
            short satindex = Raw->SdObs.SdSatObs[i].nRov;
            // 确定本颗卫星所属系统
            int sys = 0;
            sys = (Raw->SdObs.SdSatObs[i].System == GPS) ? 0 : 1;   //0 = GPS; 1 = BDS
            // 计算流动站到本颗卫星的几何距离
            double RovToSat = sqrt((RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) * (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) + (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) * (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) + (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) * (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]));

            //B矩阵
            double l = (RovPos[0] - Raw->RovEpk.SatPVT[satindex].SatPos[0]) / RovToSat - (RovPos[0] - ((sys == 0) ? RefSatPosOfGPSR[0] : RefSatPosOfBDSR[0])) / RovToRefSat[sys];
            double m = (RovPos[1] - Raw->RovEpk.SatPVT[satindex].SatPos[1]) / RovToSat - (RovPos[1] - ((sys == 0) ? RefSatPosOfGPSR[1] : RefSatPosOfBDSR[1])) / RovToRefSat[sys];
            double n = (RovPos[2] - Raw->RovEpk.SatPVT[satindex].SatPos[2]) / RovToSat - (RovPos[2] - ((sys == 0) ? RefSatPosOfGPSR[2] : RefSatPosOfBDSR[2])) / RovToRefSat[sys];
            for (int j = 0; j < 2; j++)
            {
                B(index * 2 + j, 0) = l;
                B(index * 2 + j, 1) = m;
                B(index * 2 + j, 2) = n;
            }

            //计算双差观测值和双差模糊度
            //double DDP[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double DDL[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            double DDrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];//利用前面定义的参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
            //W向量
            w(index * 2 ) = DDL[0] - DDrou - DDN(index * 2) * ((sys == 0) ? WL1_GPS : WL1_BDS);
            w(index * 2 + 1) = DDL[1] - DDrou - DDN(index * 2 + 1) * ((sys == 0) ? WL2_GPS : WL3_BDS);

            index++;
        }

        //    ③ 计算权矩阵
        //采用经验噪声设置权阵的大小
        double GP[2] = {  GPS_SatNum / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise ,  -1 / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };
        double BD[2] = {  BDS_SatNum / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise ,  -1 / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };

        //填充GPS
        for (int a = 0; a < 2 * GPS_SatNum; a++)//循环行
        {
            for (int b = 0; b < 2 * GPS_SatNum; b++)//循环列
            {
                if (b == a)//对角线上
                {
                    P(a, b) = GP[0];
                }
                else if (b % 2 == a % 2)//非对角线上
                {
                    P(a, b) = GP[1];
                }
            }
        }
        //填充BDS
        for (int a = 0; a < 2 * BDS_SatNum; a++)//循环行
        {
            for (int b = 0; b < 2 * BDS_SatNum; b++)//循环列
            {
                if (b == a)//对角线上
                {
                    P(a + 2 * GPS_SatNum, b + 2 * GPS_SatNum) = BD[0];
                }
                else if (b % 2 == a % 2)//非对角线上
                {
                    P(a + 2 * GPS_SatNum, b + 2 * GPS_SatNum) = BD[1];
                }
            }
        }

        //    6. 双差观测方程数量大于未知数，则求解   浮点解的时候已完成容错处理不需要重复进行
        
        //    7. 最小二乘解算
        MatrixXd N = B.transpose() * P * B;
        VectorXd L = B.transpose() * P * w;
        x = N.inverse() * L;
        Q_xx = N.inverse();

        //    8. 更新流动站的位置和双差模糊度参数
        RovPos[0] += x(0);
        RovPos[1] += x(1);
        RovPos[2] += x(2);

        //    9. 流动站位置增量大于阈值或迭代次数小于阈值，返回4迭代计算，否则浮点解计算完成。
        deltaNorm = sqrt(fabs(x(0)) * fabs(x(0)) + fabs(x(1)) * fabs(x(1)) + fabs(x(2)) * fabs(x(2)));
        converged = (deltaNorm < convergenceThreshold);
        iteration++;

    }

    //    10. 精度评价, 保存流动站固定解、基线向量和rms
    Raw->DDObs.dPos[0] = RovPos[0] - BasPos[0];
    Raw->DDObs.dPos[1] = RovPos[1] - BasPos[1];
    Raw->DDObs.dPos[2] = RovPos[2] - BasPos[2];

    // 流动站位置,bestpos_base+dPos
    Raw->DDObs.RovPos[0] = RovPos[0];
    Raw->DDObs.RovPos[1] = RovPos[1]; 
    Raw->DDObs.RovPos[2] = RovPos[2];

    //无bestpos_rover.Pos
    //double rms = sqrt(((RovPos[0] - Raw->bestpos_rover.Pos[0]) * (RovPos[0] - Raw->bestpos_rover.Pos[0]) + (RovPos[1] - Raw->bestpos_rover.Pos[1]) * (RovPos[1] - Raw->bestpos_rover.Pos[1]) + (RovPos[2] - Raw->bestpos_rover.Pos[2]) * (RovPos[2] - Raw->bestpos_rover.Pos[2])) / 3);
    //Raw->DDObs.FixRMS[0]=rms;
    

    return true;
}


