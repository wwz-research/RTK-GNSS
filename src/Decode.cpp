#include "SPP.h"
extern ROVERCFGINFO cfginfo;

//小端模式
short S2(char* p)
{
    short r;
    memcpy(&r, p, 2);
    return r;
}

unsigned short US2(unsigned char* p)
{
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}

int I4(char* p)
{
    int r;
    memcpy(&r, p, 4);
    return r;
}

unsigned int UI4(unsigned char* p)
{
    unsigned int r;
    memcpy(&r, p, 4);
    return r;
}

float F4(unsigned char* p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}


double D8(unsigned char* p)
{
    double r;
    memcpy(&r, p, 8);
    return r;
}

/****************************************************************************
  crc32

  目的：CRC校验

  输入参数：
   buff=缓冲区的起始地址，len=缓冲区长度
  输出参数：
   CRC校验码
****************************************************************************/
unsigned int crc32(const unsigned char* buff, int len) {
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

/****************************************************************************
  DecodeNovOem7Dat

  目的：板卡解码主函数

  输入参数：
   buff[]=缓冲区，len=缓冲区长度
  输出参数：
   obs=观测值，geph=GPS星历，beph=北斗星历，status
****************************************************************************/
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos) {
    uint16_t MessageID = 0;
    uint16_t Message_Length = 0;
    int Status = 0;          //1，观测数据；2，GPS星历；3，北斗星历；4，定位结果

    int i = 0;
    while (1)
    {
        //1. 设置循环变量i = 0，开始查找AA 44 12同步字符
        for (; i < Len; i++)
        {
            if (Buff[i] == 0xAA && Buff[i + 1] == 0x44 && Buff[i + 2] == 0x12) { break; }      //设置循环变量，查找同步字符

        }

        //2. 找到同步字符后，获取消息头长度的字符28字节， 若字节数量不足即i + 28 > len，跳出循环（break）至第6步
        if (i + 28 > Len) { break; }            //不应该判断i+28+10吗？至少要有获取Message_Length信息的长度

        //3. 从消息头中解码消息长度MsgLen和消息类型MsgID，获得整条消息buff[i, i + 28 + MsgLen + 4]，若字节数量不足即i + 28 + MsgLen + 4 > len，跳出循环（break） 至第6步
        MessageID = US2(Buff + i + 4);          // 从缓冲区中提取消息ID（第5-6字节）
        Message_Length = US2(Buff + i + 8);     // 提取消息长度（第9-10字节）
        if (i + 28 + Message_Length + 4 > Len) { break; }

        //4. CRC检验，若不通过，跳过同步字符3个字节，即i = i + 3，返回到第1步
        if (crc32(Buff + i, 28 + Message_Length) != UI4(Buff + i + 3 + 25 + Message_Length))
        {
            i += 3;
            continue;
        }

        //5. 根据消息ID，调用对应的解码函数，若解码得到观测值，跳出循环至第6步；否则，跳过整条消息，即i = i + 28 + MsgLen + 4，返回第1步
        switch (MessageID)
        {

        case 43:
            Status = decode_rangeb_oem7(Buff + i, obs);
            break;
        case 7:
            Status = decode_gpsephem(Buff + i, geph);
            break;
        case 1696:
            Status = decode_bdsephem(Buff + i, beph);
            break;
        case 42:
            Status = decode_psrpos(Buff + i, pos);
            break;
        default:
            break;
        }
        i = i + 28 + Message_Length + 4;
        if (Status == 1 && cfginfo.IsFileData == 1) break;         //解码得到观测值，跳出循环至第6步，但这里FILEMODE的作用是什么? 判断模式
    }

    memcpy(Buff + 0, Buff + i, Len - i);
    Len = Len - i;         //将Len更新为剩余字节数 ，需要返回给主函数

    return Status;
}

/****************************************************************************
  decode_rangeb_oem7

  目的：观测数据解码函数

  输入参数：
   buff=缓冲区
  输出参数：
   obs=观测值，解码成功返回值为1
****************************************************************************/
int decode_rangeb_oem7(unsigned char* Buff, EPOCHOBSDATA* obs) {
    int i, j, n, k = 0, ObsNum, Freq, Prn;
    GNSSSys sys;
    double wl;
    unsigned int ch_tr_status;
    int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SigType;
    unsigned char* p = Buff + 28;

    //1. 从消息头中解码得到观测时刻，该时刻为接收机钟表面时，用GPSTIME结构体表示。
    obs->Time.Week = US2(Buff + 14);
    obs->Time.SecOfWeek = (double)(UI4(Buff + 16) * 1.0e-3);

    //2. 解码得到观测值数量，为所有卫星所有信号观测值的总数
    ObsNum = UI4(p);
    memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));//将观测值初始化为0,清空之前的数据
    //3. 对所有信号观测值进行循环解码
    for (i = 0, p += 4; i < ObsNum; i++, p += 44)
    {
        //① 解码得到跟踪状态标记，从中取出Phase lock flag / Code lockedflag / Parity known flag / Satellite system / signal type等数据
        ch_tr_status = UI4(p + 40);
        ParityFlag = (ch_tr_status >> 11) & 0x01;//半周状态
        PhaseLockFlag = (ch_tr_status >> 10) & 0x01;//相位跟踪锁定
        CodeLockedFlag = (ch_tr_status >> 12) & 0x01;//伪距相位观测值的跟踪状态
        SatSystem = (ch_tr_status >> 16) & 0x07;//卫星类型
        SigType = (ch_tr_status >> 21) & 0x1F;//信号类型

        //② 如果卫星系统不是GPS或BDS，continue至①
        //③ 如果GPS卫星的信号类型不是L1 C / A或者L2P（Y），BDS卫星不是B1I或B3I， continue至①，并记录信号频率类型，第一频率s = 0，第二频率s = 1；
        if (SatSystem == 0)
        {
            sys = GPS;
            if (SigType == 0) {
                Freq = 0; wl = WL1_GPS;
            }
            else if (SigType == 9) { Freq = 1; wl = WL2_GPS; }
            else continue;
        }
        else if (SatSystem == 4) {
            sys = BDS;
            if (SigType == 0 || SigType == 4) {
                Freq = 0; wl = WL1_BDS;
            }
            else if (SigType == 2 || SigType == 6) { Freq = 1; wl = WL3_BDS; }
            else continue;
        }
        else continue;

        //④ 解码得到卫星号Prn以及卫星系统号，在当前观测值结构体中进行搜索，如果找到相同的卫星，将解码的观测值填充到该卫星对应的数组中；
        //   如果在当前已解码的卫星数据中没有发现，则填充到现有数据的末尾。
        Prn = US2(p);
        for (j = 0; j < MAXCHANNUM; j++) {
            if (obs->SatObs[j].System == sys && obs->SatObs[j].Prn == Prn)
            {
                n = j; break;
            }
            if (obs->SatObs[j].Prn == 0)
            {
                k = n = j; break;
            }
        }
        obs->SatObs[n].Prn = Prn;
        obs->SatObs[n].System = sys;
        obs->SatObs[n].P[Freq] = CodeLockedFlag == 1 ? D8(p + 4) : 0.0;
        obs->SatObs[n].L[Freq] = -wl * (PhaseLockFlag == 1 ? D8(p + 16) : 0.0);//注意单位和负号
        obs->SatObs[n].D[Freq] = -wl * F4(p + 28);//注意单位和负号
        obs->SatObs[n].cn[Freq] = F4(p + 32);
        obs->SatObs[n].LockTime[Freq] = F4(p + 36);
        obs->SatObs[n].half[Freq] = ParityFlag;

    }
    //4. 统计有效的卫星数量
    obs->SatNum = k + 1;

    return 1;
}

/****************************************************************************
  decode_gpsephem

  目的：GPS星历解码函数

  输入参数：
   buff=缓冲区
  输出参数：
   eph=GPS星历，解码成功返回值为2，否则返回值为0
****************************************************************************/
int decode_gpsephem(unsigned char* Buff, GPSEPHREC* geph) {
    int prn;
    unsigned char* p = Buff + 28;


    //输入buff缓冲区，buff是一条完整的GPS或BDS消息，解码得到一颗卫星的广播星历，函数返回。
    //Eph为星历数组，存储所有卫星GPS或BDS卫星的广播星历，不同卫星号的星历存储位置为数组元素i = prn - 1
    //1. 解码得到卫星号prn，判断prn-1是否超出Eph数组范围。如果越界函数返回。
    prn = UI4(p);
    if (prn < 1 || prn >= MAXGPSNUM) { return 0; }

    //2. 根据OEM星历格式，解码星历数据，注意各数据的单位。
    /*结构体
    short Prn;
    GNSSSys Sys;
    GPSTIME TOC, TOE;         //卫星钟参考时刻，星历（轨道）参考时刻
    double ClkBias, ClkDrift, ClkDriftRate;       //卫星钟的偏差、漂移、漂移速度
    double IODE, IODC;                            //星历发布时间，种的数据龄期
    double SqrtA, M0, e, OMEGA, i0, omega;        //轨道长半径平方根，平近点角，偏心率，升交点赤经，轨道倾角，近地点角距
    double Crs, Cuc, Cus, Cic, Cis, Crc;
    double DeltaN, OMEGADot, iDot;
    int SVHealth;
    double TGD1, TGD2;        //群延迟
    */
    GPSEPHREC* eph = geph + prn - 1;//确定不同卫星号的星历存储位置
    eph->Prn = prn;
    eph->Sys = GPS;
    eph->TOC.Week = eph->TOE.Week = UI4(p + 24);
    eph->TOC.SecOfWeek = D8(p + 164);
    eph->TOE.SecOfWeek = D8(p + 32);

    eph->ClkBias = D8(p + 180);
    eph->ClkDrift = D8(p + 188);
    eph->ClkDriftRate = D8(p + 196);

    eph->IODE = UI4(p + 16);
    eph->IODC = UI4(p + 160);

    eph->SqrtA = sqrt(D8(p + 40));//数据中单位是米
    eph->M0 = D8(p + 56);
    eph->e = D8(p + 64);
    eph->OMEGA = D8(p + 144);
    eph->i0 = D8(p + 128);
    eph->omega = D8(p + 72);

    eph->Crs = D8(p + 104);
    eph->Cuc = D8(p + 80);
    eph->Cus = D8(p + 88);
    eph->Cic = D8(p + 112);
    eph->Cis = D8(p + 120);
    eph->Crc = D8(p + 96);

    eph->DeltaN = D8(p + 48);
    eph->OMEGADot = D8(p + 152);
    eph->iDot = D8(p + 136);

    eph->SVHealth = UI4(p + 12);
    eph->TGD1 = D8(p + 172);
    //eph->TGD2 = D8(p + );


    return 2;
}

/****************************************************************************
  decode_bdsephem

  目的：BDS星历解码函数//?怎么处理北斗时和GPS时的差值

  输入参数：
   buff=缓冲区
  输出参数：
   eph=BDS星历，解码成功返回值为3，否则返回值为0
****************************************************************************/
int decode_bdsephem(unsigned char* Buff, GPSEPHREC* beph) {

    int prn;
    unsigned char* p = Buff + 28;


    //输入buff缓冲区，buff是一条完整的GPS或BDS消息，解码得到一颗卫星的广播星历，函数返回。
    //Eph为星历数组，存储所有卫星GPS或BDS卫星的广播星历，不同卫星号的星历存储位置为数组元素i = prn - 1
    //1. 解码得到卫星号prn，判断prn-1是否超出Eph数组范围。如果越界函数返回。
    prn = UI4(p);
    if (prn < 1 || prn >= MAXBDSNUM) { return 0; }

    //2. 根据OEM星历格式，解码星历数据，注意各数据的单位。
    /*结构体
    short Prn;
    GNSSSys Sys;
    GPSTIME TOC, TOE;         //卫星钟参考时刻，星历（轨道）参考时刻
    double ClkBias, ClkDrift, ClkDriftRate;       //卫星钟的偏差、漂移、漂移速度
    double IODE, IODC;                            //星历发布时间，种的数据龄期
    double SqrtA, M0, e, OMEGA, i0, omega;        //轨道长半径平方根，平近点角，偏心率，升交点赤经，轨道倾角，近地点角距
    double Crs, Cuc, Cus, Cic, Cis, Crc;
    double DeltaN, OMEGADot, iDot;
    int SVHealth;
    double TGD1, TGD2;        //群延迟
    */
    GPSEPHREC* eph = beph + prn - 1;//确定不同卫星号的星历存储位置
    eph->Prn = prn;
    eph->Sys = BDS;
    eph->TOC.Week = eph->TOE.Week = UI4(p + 4);

    eph->ClkBias = D8(p + 44);
    eph->ClkDrift = D8(p + 52);
    eph->ClkDriftRate = D8(p + 60);

    eph->IODE = UI4(p + 68);
    eph->IODC = UI4(p + 36);

    eph->SqrtA = D8(p + 76);
    eph->M0 = D8(p + 108);
    eph->e = D8(p + 84);
    eph->OMEGA = D8(p + 116);
    eph->i0 = D8(p + 132);
    eph->omega = D8(p + 92);

    eph->Crs = D8(p + 172);
    eph->Cuc = D8(p + 148);
    eph->Cus = D8(p + 156);
    eph->Cic = D8(p + 180);
    eph->Cis = D8(p + 188);
    eph->Crc = D8(p + 164);

    eph->DeltaN = D8(p + 100);
    eph->OMEGADot = D8(p + 124);
    eph->iDot = D8(p + 140);

    eph->SVHealth = UI4(p + 16);
    eph->TGD1 = D8(p + 20);
    eph->TGD2 = D8(p + 28);

    //3. BDS星历的参考时间TOC和TOE均定义为GPSTIME结构体，注意它们的时间系统。
    eph->TOC.SecOfWeek = UI4(p + 40);
    eph->TOE.SecOfWeek = UI4(p + 72);

    return 3;
}

/****************************************************************************
  decode_psrpos

  目的：定位结果数据提取

  输入参数：
   buff=缓冲区
  输出参数：
   pos=定位结果，解码成功返回值为4，否则返回值为0
****************************************************************************/
int decode_psrpos(unsigned char* Buff, POSRES* pos) {
    pos->Time.Week = US2(Buff + 14);
    pos->Time.SecOfWeek = (double)(UI4(Buff + 16) * 1.0e-3);

    unsigned char* p = Buff + 28;
    double BLH[3];
    BLH[0] = D8(p + 16) * Rad;//经度
    BLH[1] = D8(p + 8) * Rad;//纬度
    BLH[2] = D8(p + 24) + F4(p + 32);

    BLHToXYZ(BLH, pos->Pos, R_WGS84, F_WGS84);

    return 4;
}

