#include "SPP.h"
extern ROVERCFGINFO cfginfo;

/************************************************************
函数名：滤波初始化函数

目的：初始化滤波的状态向量X（流动站的位置参数与双差N）其对应的方差协方差矩阵P

参数：Raw    指向历元总数据的指针
      Base   基站的SPP结果
      Rov    流动站的SPP结果
      ekf    滤波数据
************************************************************/
bool InitFilter(RAWDAT* Raw, POSRES* Base, POSRES* Rov, RTKEKF* ekf)
{
    int i, j;

    if (Base->IsSuccess == false || Rov->IsSuccess == false)
    {
        cout << "SPP解算失败不进行相对定位EKF计算" << endl;
        return false;
    }
    if (Raw->DDObs.RefPos[0] == -1 || Raw->DDObs.RefPos[1] == -1)
    {
        cout << "参考星选取失败不进行相对定位EKF计算" << endl;
        return false;
    }
    
    // 状态清零
    ekf->X.setZero();
    ekf->X0.setZero();
    ekf->P.setZero();
    ekf->P0.setZero();

    // 初始化时间标记
    memcpy(&ekf->Time, &Raw->SdObs.Time, sizeof(GPSTIME));
    // 初始化状态的位置元素:使用流动站SPP结果
    ekf->X0(0) = Rov->Pos[0];
    ekf->X0(1) = Rov->Pos[1];
    ekf->X0(2) = Rov->Pos[2];
    //cout << ekf->X0(0) << "  " << ekf->X0(1) << "  " << ekf->X0(2) << "  ";
    // 初始化状态的双差模糊度元素：用双差伪距和双差相位的差值（遍历单差数据，按顺序计算并填充）
    int index = 0;    //记录可用的双差观测
    for (i = 0; i < Raw->SdObs.SatNum; i++)
    {
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1])
        {
            //ekf->Index[i] = -1;
            continue; // 基准星不计入可用双差卫星数
        }
        else if (Raw->SdObs.SdSatObs[i].Valid == false || Raw->SdObs.SdSatObs[i].half[0] == 0 || Raw->SdObs.SdSatObs[i].half[1] == 0)
        {
            //ekf->Index[i] = -1;
            continue; // 跳过粗差和半周
        }
        else 
        {
            // 确定本颗卫星所属系统
            int sys = 0;
            sys = (Raw->SdObs.SdSatObs[i].System == GPS) ? 0 : 1;   //0 = GPS; 1 = BDS

            //计算双差观测值和双差模糊度
            double DDP[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double DDL[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            ekf->X0(index * 2 + 3) = (DDL[0] - DDP[0]) / ((sys == 0) ? WL1_GPS : WL1_BDS);
            ekf->X0(index * 2 + 4) = (DDL[1] - DDP[1]) / ((sys == 0) ? WL2_GPS : WL3_BDS);
            //cout << ekf->X0(index * 2 + 3) << "  " << ekf->X0(index * 2 + 4) << "  ";

            //ekf->Index[i] = index; // 本颗卫星对应站间单差数据产生的双频双差模糊度存放在状态X中的索引为index,index+1

            index++;
        }
    }


    // 初始化状态协方差
    ekf->P0(0, 0) = 100;
    ekf->P0(1, 1) = 100;
    ekf->P0(2, 2) = 100;

    for (j = 0; j < index; j++)
    {
        ekf->P0(j * 2 + 3, j * 2 + 3) = 500;
        ekf->P0(j * 2 + 4, j * 2 + 4) = 500;
    }
    
    /*for (j = 0; j < index; j++)
    {
        if (j < 3) ekf->P0(j , j ) = 100;
        else ekf->P0(j , j) = 500;               //(3-6)^2
    }*/          //         !索引错了

    //cout <<"初始化可用双差卫星数" << index << endl;

    // 滤波初始化完成，更新标记
    ekf->IsInit = true;
    return true;
}

/************************************************************
函数名：滤波时间更新

目的：滤波时间更新

参数：Raw    指向历元总数据的指针
      Rov    流动站的SPP结果
      ekf    滤波数据

注意：
    卫星升降，状态与状态协方差矩阵重置
    基准星变化，模糊度及其协方差的重整
    周跳探测，模糊度参数重置与更新
    状态参数与协方差矩阵的时间更新
************************************************************/
bool EkfPredict(RAWDAT* Raw, POSRES* Rov, RTKEKF* ekf)
{
    //int i, j, k, a, b, m, n, c, d;

    //时间传递
    memcpy(&ekf->Time, &Raw->SdObs.Time, sizeof(GPSTIME));

    //1. 统计双差卫星数，定义矩阵fai和Q
    //统计上一历元的可用双差卫星数，先初始化为0，避免之前历元数据的影响
    ekf->DDObs.Sats = 0;//ekf->DDObs、ekf->SDObs存储上一历元的双差、单差
    for (int i = 0; i < ekf->SDObs.SatNum; i++)
    {
        if (i == ekf->DDObs.RefPos[0] || i == ekf->DDObs.RefPos[1]) continue;
        else if (ekf->SDObs.SdSatObs[i].Valid == false || ekf->SDObs.SdSatObs[i].half[0] == 0 || ekf->SDObs.SdSatObs[i].half[1] == 0) continue;
        else ekf->DDObs.Sats++;
    }

    //统计当前历元的可用双差卫星数：ekf->CurDDObs数据是从主函数中拷贝Raw->DDObs得到的
    ekf->CurDDObs.Sats = 0;//ekf->CurDDObs、Raw->SdObs存储当前历元的双差、单差
    for (int j = 0; j < Raw->SdObs.SatNum; j++)
    {
        if (j == ekf->CurDDObs.RefPos[0] || j == ekf->CurDDObs.RefPos[1]) continue;
        else if (Raw->SdObs.SdSatObs[j].Valid == false || Raw->SdObs.SdSatObs[j].half[0] == 0 || Raw->SdObs.SdSatObs[j].half[1] == 0) continue;
        else ekf->CurDDObs.Sats++;
    }

    //根据双差卫星数用eigen定义状态转移矩阵，预测状态的误差协方差并初始化为零
    Eigen::MatrixXd fai = Eigen::MatrixXd::Zero(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->DDObs.Sats);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats);
    Eigen::VectorXd X = Eigen::VectorXd::Zero(3 + 2 * ekf->CurDDObs.Sats);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats);     //注意为方差协方差矩阵而非权阵
    //将X0和P0用上一历元的数据填充
    Eigen::VectorXd X0 = ekf->X0.head( 3 + 2 * ekf->DDObs.Sats);
    Eigen::MatrixXd P0 = ekf->P0.topLeftCorner( 3 + 2 * ekf->DDObs.Sats, 3 + 2 * ekf->DDObs.Sats);

    //2. 填充位置元素对应的fai和Q  
    for (int k = 0; k < 3; k++)
    {
        fai(k, k) = 1;
        Q(k, k) = 100;   //SPP定位，设置为5-10m，为啥设置成1e-4 * 1e-4  ?
    }

    //3. 填充模糊度对应的fai和Q
    //① 记录上一历元与当前历元双系统的参考星有无发生变化 0：GPS 1：BDS
    bool RefIsChange[2] = { false, false };
    for (int a = 0; a < 2; a++)
    {
        if (ekf->CurDDObs.RefPrn[a] != ekf->DDObs.RefPrn[a]) RefIsChange[a] = true;
        else continue;
    }

    //② 循环上一历元单差数据，记录当前历元的参考星在上一历元中是否可用
    bool CurRefIsValid[2] = { false,false };
    int CurRefIndex[2] = { 0, 0 };//当前历元参考星在上一历元双差数据中的存储位置
    int CurRefIndexNum=0;
    for (int b = 0; b < ekf->SDObs.SatNum; b++)
    {
        //判断有效性，不满足跳过
        if (ekf->SDObs.SdSatObs[b].Valid == false || ekf->SDObs.SdSatObs[b].half[0] == 0 || ekf->SDObs.SdSatObs[b].half[1] == 0) continue;

        //判断系统和Prn是否匹配
        if (ekf->SDObs.SdSatObs[b].System == GPS && ekf->SDObs.SdSatObs[b].Prn == ekf->CurDDObs.RefPrn[0])
        {
            CurRefIsValid[0] = true;
            CurRefIndex[0] = CurRefIndexNum;//当前历元参考星在上一历元单差数据中的存储位置
            CurRefIndexNum++;
        }
        else if (ekf->SDObs.SdSatObs[b].System == BDS && ekf->SDObs.SdSatObs[b].Prn == ekf->CurDDObs.RefPrn[1])
        {
            CurRefIsValid[1] = true;
            CurRefIndex[1] = CurRefIndexNum;
            CurRefIndexNum;
        }
        else continue;
    }

    //③ 进行填充
    short sys = 0;    //卫星系统
    int rowindex = 3;
    int colindex = 3;
    int curindex=0;
    int initialize_index[MAXCHANNUM];//当前历元中双差模糊度需要初始化的卫星
    bool CurSatIsFind = false;   //标记是否是新升起的卫星，即在上一个历元是否找到当前历元的卫星
    memset(initialize_index, 0, sizeof(initialize_index));//初始化为0，如果卫星需要初始化则赋值为1

    for (int m = 0; m < Raw->SdObs.SatNum; m++)//遍历当前历元
    {
        
        if (m == ekf->CurDDObs.RefPos[0] || m == ekf->CurDDObs.RefPos[1]) continue;      //!ekf->CurDDObs和 Raw->DDObs等效
        else if (Raw->SdObs.SdSatObs[m].Valid == false || Raw->SdObs.SdSatObs[m].half[0] == 0 || Raw->SdObs.SdSatObs[m].half[1] == 0) continue;
        else;

        CurSatIsFind = false;
        for (int n = 0; n < ekf->SDObs.SatNum; n++)//遍历上一历元
        {
            if (n == ekf->DDObs.RefPos[0] || n == ekf->DDObs.RefPos[1]) continue;        //！判断条件不要写错了
            else if (ekf->SDObs.SdSatObs[n].Valid == false || ekf->SDObs.SdSatObs[n].half[0] == 0 || ekf->SDObs.SdSatObs[n].half[1] == 0) continue;
            else;
            
            sys = (ekf->SDObs.SdSatObs[n].System == GPS) ? 0 : 1;
            
            //a.在上一个历元找到当前历元的卫星
            if (ekf->SDObs.SdSatObs[n].System == Raw->SdObs.SdSatObs[m].System && ekf->SDObs.SdSatObs[n].Prn == Raw->SdObs.SdSatObs[m].Prn)
            {
                CurSatIsFind = true;

                //b.没有周跳，模糊度不变，设置微小量e^-10
                if (ekf->SDObs.SdSatObs[n].Valid == true && ekf->SDObs.SdSatObs[n].half[0] == 1 && ekf->SDObs.SdSatObs[n].half[1] == 1)
                {
                    //c.参考星未改变，直接继承  矩阵的索引需要确定一下
                    if (RefIsChange[sys] == false)
                    {
                        fai(rowindex, colindex) = 1;
                        fai(rowindex + 1, colindex + 1) = 1;
                        Q(rowindex, rowindex) = 1e-10;
                        Q(rowindex + 1, rowindex + 1) = 1e-10;
                    }

                    //c.参考星改变
                    else
                    {
                        //d.当前历元参考星在上一历元中可用，间接继承
                        if (CurRefIsValid[sys] == true)
                        {
                            fai(rowindex, colindex) = 1;
                            fai(rowindex + 1, colindex + 1) = 1;
                            fai(rowindex, 3 + 2 * CurRefIndex[sys]) = -1;           //感觉只有这用到了，不是很必要，注意ekf->Index[CurRefIndex[sys]]的更新
                            fai(rowindex + 1, 3 + 2 * CurRefIndex[sys] + 1) = -1;
                            Q(rowindex, rowindex) = 1e-10;
                            Q(rowindex + 1, rowindex + 1) = 1e-10;
                        }

                        //d.当前历元参考星在上一历元中不可用，重新初始化
                        //其中，fai中的双差模糊度的系数默认为零，不使用上一历元的双差模糊度进行直接或者间接的传递
                        else
                        {
                            Q(rowindex, rowindex) = 500;
                            Q(rowindex + 1, rowindex + 1) = 500;
                            initialize_index[curindex] = 1;
                        }
                    }
                }

                //b.检测到周跳，代估参数和误差协方差重新初始化
                //其中，fai中的双差模糊度的系数默认为零，不使用上一历元的双差模糊度进行直接或者间接的传递
                else
                {
                    Q(rowindex, rowindex) = 500;
                    Q(rowindex + 1, rowindex + 1) = 500;
                    initialize_index[curindex] = 1;
                }

                //列索引更新
                colindex += 2;


            }
            //a.继续寻找         !开始的时候在这里就进行“未找到”的条件处理，逻辑错误导致数组也超限
            else continue;
        }
        //a.未找到，新升起卫星
        //其中，fai中的双差模糊度的系数默认为零，无法找到上一历元的双差模糊度进行直接或者间接的传递
        if(CurSatIsFind == false)
        {
            Q(rowindex, rowindex) = 500;
            Q(rowindex + 1, rowindex + 1) = 500;
            initialize_index[curindex] = 1;
        }

        //行索引更新，当前历元有效卫星数更新
        rowindex += 2;
        curindex++;
    }

    //4. 进行时间更新
    X = fai * X0;
    P = fai * P0 * fai.transpose() + Q;     //Q的参数呢？单位阵

    //5. 将位置参数（用当前历元SPP结果）以及需要重新初始化的双差模糊度进行初始化
    X(0) = Rov->Pos[0];
    X(1) = Rov->Pos[1];
    X(2) = Rov->Pos[2];

    curindex = 0;
    for (int c = 0; c < Raw->SdObs.SatNum; c++)
    {
        if (c == Raw->DDObs.RefPos[0] || c == Raw->DDObs.RefPos[1]) continue;
        else if (Raw->SdObs.SdSatObs[c].Valid == false || Raw->SdObs.SdSatObs[c].half[0] == 0 || Raw->SdObs.SdSatObs[c].half[1] == 0) continue;
        else;

        sys = (ekf->SDObs.SdSatObs[c].System == GPS) ? 0 : 1;

        //根据索引判断需要重新初始化的双差模糊度
        if (initialize_index[curindex] == 1)
        {
            double DDP[2] = { Raw->SdObs.SdSatObs[c].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[c].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
            double DDL[2] = { Raw->SdObs.SdSatObs[c].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[c].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
            X(curindex * 2 + 3) = (DDL[0] - DDP[0]) / ((sys == 0) ? WL1_GPS : WL1_BDS);
            X(curindex * 2 + 4) = (DDL[1] - DDP[1]) / ((sys == 0) ? WL2_GPS : WL3_BDS);

        }
        curindex++;
    }

    //fai和Q可以打印出来看一下
    //SaveEigen(fai, "fai_时间更新.txt");
    //SaveEigen(P0, "P0_时间更新.txt");
    //SaveEigen(Q, "Q_时间更新.txt");
    //SaveEigen(P, "P_时间更新.txt");

    //结束时间更新：更新X和P；
    ekf->X.setZero();
    ekf->P.setZero();
    ekf->X.head(3 + 2 * ekf->CurDDObs.Sats) = X;
    ekf->P.topLeftCorner(3 + 2 * ekf->CurDDObs.Sats, 3 + 2 * ekf->CurDDObs.Sats) = P;

    //将ekf->index更新为当前历元，作为下一历元的索引
    //curindex = 0;
    
    //for (int d = 0; d < Raw->SdObs.SatNum; d++)
    //{
    //    if (d == Raw->DDObs.RefPos[0] || d == Raw->DDObs.RefPos[1])
    //    {
    //        ekf->Index[d] = -1;
    //        continue; // 基准星不计入可用双差卫星数
    //    }
    //    else if (Raw->SdObs.SdSatObs[d].Valid == false || Raw->SdObs.SdSatObs[d].half[0] == 0 || Raw->SdObs.SdSatObs[d].half[1] == 0)
    //    {
    //        ekf->Index[d] = -1;
    //        continue; // 跳过粗差和半周
    //    }
    //    else
    //    {
    //        ekf->Index[d] = curindex;
    //        curindex++;
    //    }
    //}

    return true;
}

/************************************************************
函数名：滤波测量更新

目的：滤波测量更新

参数：Raw    指向历元总数据的指针
      ekf    滤波数据

观测方程线性化，得到观测矩阵B、残差向量W、观测噪声矩阵R
    
************************************************************/
bool EkfMeasureUpdate(RAWDAT* Raw,RTKEKF* ekf)
{
    /*浮点解进行测量更新*/
    //    1. 设置基站和流动站位置初值（基站为解码得到的值，流动站为时间更新得到的值）
    double BasPos[3] = { Raw->bestpos_base.Pos[0],Raw->bestpos_base.Pos[1],Raw->bestpos_base.Pos[2] };
    double RovPos[3] = { ekf->X(0),ekf->X(1),ekf->X(2) };

    //    2. 计算GPS和BDS双差卫星数（该程序为双系统双频；若为单双频混用：各系统每个频率的双差卫星数）
    short SatNum = 0, GPS_SatNum = 0, BDS_SatNum = 0;
    if (Raw->SdObs.SatNum < 2) return false;//单差观测值小于2，无法进行双差
    for (int m = 0; m < Raw->SdObs.SatNum; m++)
    {
        if (m == Raw->DDObs.RefPos[0] || m == Raw->DDObs.RefPos[1]) continue; // 基准星不计入可用双差卫星数
        if (Raw->SdObs.SdSatObs[m].Valid == false || Raw->SdObs.SdSatObs[m].half[0] == 0 || Raw->SdObs.SdSatObs[m].half[1] == 0)continue;

        if (Raw->SdObs.SdSatObs[m].System == GPS) GPS_SatNum++;
        else BDS_SatNum++;
    }
    SatNum = GPS_SatNum + BDS_SatNum;//和时间更新中计算得到的ekf->CurDDObs.Sats应该是一样的

    //根据双差卫星数定义矩阵变量
    int row = 4 * SatNum;       //行，方程个数
    int col = 3 + 2 * SatNum;   //列，未知数个数
    double BasToSats[MAXCHANNUM] = { 0 };// 基站坐标到所有卫星的几何距离（下标与基站原始观测数据的索引一致），大小可以设置为Raw->BasEpk.SatNum/MAXCHANNUM
    double RovToRefSat[2] = { 0,0 };// 2个系统流动站到参考星的距离
    //Eigen::VectorXd DDN = Eigen::VectorXd::Zero(col - 3); // 双差模糊度
    Eigen::VectorXd w = Eigen::VectorXd::Zero(row);          // 残差向量W
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(row, col); // 设计矩阵
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(row, row); // 测量权阵，即观测噪声矩阵R
    Eigen::VectorXd x_k = Eigen::VectorXd::Zero(col);          // 待估参数改正数
    Eigen::MatrixXd P_k = Eigen::MatrixXd::Zero(col, col); // 测量更新得到的协方差矩阵，不是权阵！
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(col, col);
    Eigen::MatrixXd Q_xx = Eigen::MatrixXd::Zero(col, col); // 协因数阵
    //Eigen::VectorXd V = Eigen::VectorXd::Zero(row);          // V=Bx-w，计算残差平方和rms
    //提取时间更新得到的x、P到x0和P0中
    Eigen::VectorXd x0 = ekf->X.head(col);
    Eigen::MatrixXd P0 = ekf->P.topLeftCorner(col, col);


    //    3. 计算基站坐标到所有卫星的几何距离，存储到数组中；得到参考卫星位置；得到流动站到参考卫星的位置
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
    // 计算流动站到参考卫星的距离
    RovToRefSat[0] = sqrt((RovPos[0] - RefSatPosOfGPSR[0]) * (RovPos[0] - RefSatPosOfGPSR[0]) + (RovPos[1] - RefSatPosOfGPSR[1]) * (RovPos[1] - RefSatPosOfGPSR[1]) + (RovPos[2] - RefSatPosOfGPSR[2]) * (RovPos[2] - RefSatPosOfGPSR[2]));
    RovToRefSat[1] = sqrt((RovPos[0] - RefSatPosOfBDSR[0]) * (RovPos[0] - RefSatPosOfBDSR[0]) + (RovPos[1] - RefSatPosOfBDSR[1]) * (RovPos[1] - RefSatPosOfBDSR[1]) + (RovPos[2] - RefSatPosOfBDSR[2]) * (RovPos[2] - RefSatPosOfBDSR[2]));

    //     4. 对单差数据进行循环填充矩阵
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
        //DDN(index * 2) = (DDL[0] - DDP[0]) / ((sys == 0) ? WL1_GPS : WL1_BDS);
        //DDN(index * 2 + 1) = (DDL[1] - DDP[1]) / ((sys == 0) ? WL2_GPS : WL3_BDS);
        //W向量
        w(index * 4) = DDP[0] - DDrou;
        w(index * 4 + 1) = DDP[1] - DDrou;
        w(index * 4 + 2) = DDL[0] - DDrou - x0(3+index * 2) * ((sys == 0) ? WL1_GPS : WL1_BDS);
        w(index * 4 + 3) = DDL[1] - DDrou - x0(3+index * 2 + 1) * ((sys == 0) ? WL2_GPS : WL3_BDS);

        index++;
    }
    //可以输出看一下SatNum和index的大小是不是相等的，按理来说应该是相等的
    ekf->nSats = SatNum;  // ekf->nSats = index;

    //填充R矩阵,P的逆阵
    
    //P是权阵，后面要求逆
    double GP[4] = { GPS_SatNum / (GPS_SatNum + 1.0) / 2 / cfginfo.CodeNoise/ cfginfo.CodeNoise , GPS_SatNum / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise, -1 / (GPS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise, -1 / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };
    double BD[4] = { BDS_SatNum / (BDS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise, BDS_SatNum / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise, -1 / (BDS_SatNum + 1.0) / 2 / cfginfo.CodeNoise / cfginfo.CodeNoise, -1 / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };

    //P是协方差阵
    //double GP[4] = { 4 * cfginfo.CodeNoise* cfginfo.CodeNoise, 4 * cfginfo.CPNoise * cfginfo.CPNoise, 2 *  cfginfo.CodeNoise* cfginfo.CodeNoise, 2 * cfginfo.CPNoise * cfginfo.CPNoise };
    //double BD[4] = { 4 * cfginfo.CodeNoise* cfginfo.CodeNoise, 4 * cfginfo.CPNoise * cfginfo.CPNoise, 2 *  cfginfo.CodeNoise* cfginfo.CodeNoise, 2 * cfginfo.CPNoise * cfginfo.CPNoise };

    //填充GPS
    for (int a = 0; a < 4 * GPS_SatNum; a++)//循环行
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
                if (a % 4 == 0 || a % 4 == 1)  P(a, b) = GP[2];
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
                if (a % 4 == 0 || a % 4 == 1) P(a + 4 * GPS_SatNum, b + 4 * GPS_SatNum) = BD[0];
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

    //    7. 进行测量更新
    MatrixXd K_k = P0 * B.transpose() * (B * P0 * B.transpose() + P.inverse()).inverse();
    VectorXd K_kw= K_k * w;
    x_k = x0 + K_k * w;             //x_k = x0 + K_k*(w - B * x0);是不对的
    P_k = (I - K_k * B) * P0 * (I - K_k * B).transpose() + K_k * P * K_k.transpose();
    Q_xx = (B.transpose() * P * B).inverse();

    //存储浮点解基线向量
    Raw->DDObs.dPos_float[0] = x_k(0) - BasPos[0];
    Raw->DDObs.dPos_float[1] = x_k(1) - BasPos[1];
    Raw->DDObs.dPos_float[2] = x_k(2) - BasPos[2];

    /*
    cout << "x0: ";
    for (int c = 0; c < col; c++)
    {
        cout << fixed << setprecision(3) << x0(c) << " ";
    }
    cout << endl;

    cout << "x_k: ";
    for (int c = 0; c < col; c++)
    {
        cout << fixed << setprecision(3) << x_k(c) << " ";
    }
    cout << endl;

    cout << "x_k-x0: ";
    for (int c = 0; c < col; c++)
    {
        cout << fixed << setprecision(3) << (x_k(c) - x0(c)) << " ";
    }
    cout << endl;
    */

    //SaveEigen(B, "B.txt");        // 保存设计矩阵 B (row x col)
    //SaveEigen(w, "w.txt");        // 保存残差向量 w (row x 1)
    //SaveEigen(P, "P.txt");        // 保存 P (row x row)
    //SaveEigen(P0, "P0.txt");        // 保存 P (row x row)
    //SaveEigen(K_k, "K_k");
    //SaveEigen(K_kw, "K_kw");

    /*模糊度固定*/
    //模糊度的协因数阵
    int amb_dim = 2 * SatNum;  // 模糊度参数维数
    Eigen::MatrixXd Q_nn = Q_xx.block(3, 3, amb_dim, amb_dim);// 取出模糊度协因数子矩阵（右下角部分）
    double N_k[MAXCHANNUM * 2] = { 0 };
    double Q_k[(MAXCHANNUM * 2) * (MAXCHANNUM * 2)] = { 0 };
    for (int c = 0; c < amb_dim; c++)
    {
        N_k[c] = x_k(c + 3);
        for (int d = 0; d < amb_dim; d++)
        {
            Q_k[c * amb_dim + d] = Q_nn(c, d);
        }
    }

    double FixedAmb[MAXCHANNUM * 4];
    double ResAmb[2];
    if (lambda(amb_dim, 2, N_k, Q_k, FixedAmb, ResAmb) != 0)
    {
        //cout << "历元：" << Raw->SdObs.Time.SecOfWeek << "模糊度固定失败";
        return false;
    }
    else
    {
        //cout << "历元：" << Raw->SdObs.Time.SecOfWeek << "模糊度固定成功";
    }
    Raw->DDObs.Ratio = ResAmb[1] / ResAmb[0];
    //cout << fixed << setprecision(3) << " ratio：" << Raw->DDObs.Ratio;
    if (Raw->DDObs.Ratio > cfginfo.RatioThres) Raw->DDObs.bFixed = true;
    else Raw->DDObs.bFixed = false;

    /*固定解进行测量更新*/
    int row_fix = 2 * SatNum;       //行，方程个数
    int col_fix = 3;
    Eigen::VectorXd w_fix = Eigen::VectorXd::Zero(row_fix);          // 
    Eigen::MatrixXd B_fix = Eigen::MatrixXd::Zero(row_fix, col_fix); // 设计矩阵
    Eigen::MatrixXd P_fix = Eigen::MatrixXd::Zero(row_fix, row_fix); // 测量权阵
    Eigen::VectorXd x_k_fix = Eigen::VectorXd::Zero(col_fix);
    Eigen::MatrixXd P_k_fix = Eigen::MatrixXd::Zero(col_fix, col_fix); // 测量更新得到的协方差矩阵，不是权阵！
    Eigen::MatrixXd I_fix = Eigen::MatrixXd::Identity(col_fix, col_fix);
    Eigen::MatrixXd Q_xx_fix = Eigen::MatrixXd::Zero(col_fix, col_fix); // 协因数阵
    //Eigen::VectorXd V_fix = Eigen::VectorXd::Zero(row);          // V=Bx-w，计算残差平方和rms
    //重新提取时间更新得到的x、P
    Eigen::VectorXd x0_fix = ekf->X.head(col_fix);
    Eigen::MatrixXd P0_fix = ekf->P.topLeftCorner(col_fix, col_fix);

    //     4. 对单差数据进行循环填充矩阵
    index = 0;    //记录可用的双差观测
    for (int i = 0; i < Raw->SdObs.SatNum; i++)
    {
        //    ① 参考星不用计算，存在半周不参与计算
        if (i == Raw->DDObs.RefPos[0] || i == Raw->DDObs.RefPos[1]) continue;
        else if (Raw->SdObs.SdSatObs[i].Valid == false || Raw->SdObs.SdSatObs[i].half[0] == 0 || Raw->SdObs.SdSatObs[i].half[1] == 0)continue;
        else;

        //    ② 线性化双差观测方程得到B_fix矩阵和W_fix向量

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
            B_fix(index * 2 + j, 0) = l;
            B_fix(index * 2 + j, 1) = m;
            B_fix(index * 2 + j, 2) = n;
        }

        //计算双差观测值和双差模糊度
        //double DDP[2] = { Raw->SdObs.SdSatObs[i].dP[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[0], Raw->SdObs.SdSatObs[i].dP[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dP[1] };
        double DDL[2] = { Raw->SdObs.SdSatObs[i].dL[0] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[0], Raw->SdObs.SdSatObs[i].dL[1] - Raw->SdObs.SdSatObs[Raw->DDObs.RefPos[sys]].dL[1] };
        double DDrou = RovToSat - RovToRefSat[sys] + BasToSats[RefSatOfBas[sys]] - BasToSats[Raw->SdObs.SdSatObs[i].nBas];//利用前面定义的参考卫星位置在原始观测数据中的索引 0=GPS, 1=BDS
        //W向量
        w_fix(index * 2) = DDL[0] - DDrou - FixedAmb[index * 2] * ((sys == 0) ? WL1_GPS : WL1_BDS);
        w_fix(index * 2 + 1) = DDL[1] - DDrou - FixedAmb[index * 2 + 1] * ((sys == 0) ? WL2_GPS : WL3_BDS);

        index++;
    }

    //    ③ 计算权矩阵
    //采用经验噪声设置权阵的大小
    double GP_fix[2] = { GPS_SatNum / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise ,  -1 / (GPS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };
    double BD_fix[2] = { BDS_SatNum / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise ,  -1 / (BDS_SatNum + 1.0) / 2 / cfginfo.CPNoise / cfginfo.CPNoise };

    //填充GPS
    for (int a = 0; a < 2 * GPS_SatNum; a++)//循环行
    {
        for (int b = 0; b < 2 * GPS_SatNum; b++)//循环列
        {
            if (b == a)//对角线上
            {
                P_fix(a, b) = GP_fix[0];
            }
            else if (b % 2 == a % 2)//非对角线上
            {
                P_fix(a, b) = GP_fix[1];
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
                P_fix(a + 2 * GPS_SatNum, b + 2 * GPS_SatNum) = BD_fix[0];
            }
            else if (b % 2 == a % 2)//非对角线上
            {
                P_fix(a + 2 * GPS_SatNum, b + 2 * GPS_SatNum) = BD_fix[1];
            }
        }
    }

    //
    MatrixXd K_k_fix = P0_fix * B_fix.transpose() * (B_fix * P0_fix * B_fix.transpose() + P_fix.inverse()).inverse();//P0也要更新
    x_k_fix = x0_fix + K_k_fix * w_fix ;                 //x_k_fix = x0_fix + K_k_fix * (w_fix - B_fix * x0_fix); 错的
    P_k_fix = (I_fix - K_k_fix * B_fix) * P0_fix * (I_fix - K_k_fix * B_fix).transpose() + K_k_fix * P_fix * K_k_fix.transpose();
    Q_xx_fix = (B_fix.transpose() * P_fix * B_fix).inverse();


    // 精度评价, 保存流动站固定解、基线向量和rms、sigmaP、模糊度固定解(最优和次优）
    Raw->DDObs.Sats = SatNum;
    Raw->DDObs.DDSatNum[0] = GPS_SatNum;
    Raw->DDObs.DDSatNum[1] = BDS_SatNum;
    Raw->DDObs.dPos[0] = x_k_fix(0) - BasPos[0];
    Raw->DDObs.dPos[1] = x_k_fix(1) - BasPos[1];
    Raw->DDObs.dPos[2] = x_k_fix(2) - BasPos[2];

    // 流动站位置,bestpos_base+dPos
    Raw->DDObs.RovPos[0] = x_k_fix(0);
    Raw->DDObs.RovPos[1] = x_k_fix(1);
    Raw->DDObs.RovPos[2] = x_k_fix(2);

    //double rms = sqrt(((Raw->DDObs.RovPos[0] - Raw->bestpos_rover.Pos[0]) * (Raw->DDObs.RovPos[0] - Raw->bestpos_rover.Pos[0]) + (Raw->DDObs.RovPos[1] - Raw->bestpos_rover.Pos[1]) * (Raw->DDObs.RovPos[1] - Raw->bestpos_rover.Pos[1]) + (Raw->DDObs.RovPos[2] - Raw->bestpos_rover.Pos[2]) * (Raw->DDObs.RovPos[2] - Raw->bestpos_rover.Pos[2])) / 3);
    //Raw->DDObs.FixRMS[0] = rms;
    Raw->DDObs.PDOP = sqrt(Q_xx(0, 0) + Q_xx(1, 1) + Q_xx(2, 2));
    memcpy(Raw->DDObs.FixedAmb, FixedAmb, sizeof(FixedAmb));

    //double Rblh_ekf[3];//流动站blh结果
    //XYZToBLH(Raw->DDObs.RovPos, Rblh_ekf, R_WGS84, F_WGS84);
    //cout << fixed << setprecision(8) << "B= " << (Rblh_ekf[1] * Deg) << "  " << " L= " << (Rblh_ekf[0] * Deg) << "  " << fixed << setprecision(3) << "H= " << Rblh_ekf[2] << "  " 
    //     <<x_k_fix(0) << "  " << x_k_fix(1) << "  " << x_k_fix(2) << "  " ;

    //for (int c = 0; c < amb_dim; c++)
    //{
    //    cout << FixedAmb[c] << "  ";
    //}
    //cout << endl;

    //结束测量更新：更新ekf的X0和P0
    ekf->X0.setZero();
    ekf->P0.setZero();
    ekf->X0.head(col) = x_k;
    ekf->P0.topLeftCorner(col, col) = P_k;

    return true;
}