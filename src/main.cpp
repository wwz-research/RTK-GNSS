#include "SPP.h"
#include "socket.h"   

// 配置信息变量
ROVERCFGINFO cfginfo;

int main()
{
    // 读取配置信息
    if (!ReadRTKConfigInfo("Config.txt", cfginfo))
    {
        printf("读取配置文件失败！\n");
        return 0;
    }
    else printf("读取配置文件成功！\n");

    //打开基站和流动站文件
    FILE* fbase = nullptr;
    errno_t err_base = fopen_s(&fbase, cfginfo.BasObsDatFile, "rb");
    if (err_base != 0 || fbase == nullptr)
    {
        printf("Cannot open basefile. Error code: %d\n", err_base);
        return 0;
    }
    FILE* frover = nullptr;
    errno_t err_rover = fopen_s(&frover, cfginfo.RovObsDatFile, "rb");
    if (err_rover != 0 || frover == nullptr)
    {
        printf("Cannot open roverfile. Error code: %d\n", err_rover);
        fclose(fbase);  // 关闭已打开的base文件
        return 0;
    }

    //打开结果输出文件
    //ofstream outfile("output_file.txt");
    ofstream outfile(cfginfo.ResFile);

    //打开基站和流动站两个网口
    SOCKET BasNet;
    if (OpenSocket(&BasNet, cfginfo.BasNetIP, cfginfo.BasNetPort) == false)
    {
        printf("Base ip & port was not opened.\n");
        return 0;
    }
    SOCKET RovNet;
    if (OpenSocket(&RovNet, cfginfo.RovNetIP, cfginfo.RovNetPort) == false)
    {
        printf("Rover ip & port was not opened.\n");
        return 0;
    }

    
    POSRES Bres;      // 基站的SPP结果
    POSRES Rres;      // 流动站的SPP结果
    RAWDAT Raw;       // RTK结构体
    FloatResult Fres; // LSQ浮点解结果
    RTKEKF ekf;       // 滤波数据
    double Rres_RTK[3];    //RTK：流动站xyz结果
    double Rblh_RTK[3];    //RTK：流动站blh结果  
    double Rres_RTKfloat[3];    //RTK：流动站xyz浮点解结果
    double Rblh_RTKfloat[3];    //RTK：流动站blh浮点解结果  

    while (1)
    {
       
        // 时间同步
        int SynFlag;
        if (cfginfo.IsFileData == 1)
        {
            //cout << "文件模式进行RTK定位" << endl;
            SynFlag = TimeSyn(fbase, frover, &Raw);//文件模式
        }
        else if (cfginfo.IsFileData == 0)
        {
            //cout << "串口模式进行RTK定位" << endl;
            SynFlag = TimeSynSoc(BasNet, RovNet, &Raw);//串口模式
        }
        else cout << "非文件或者串口模式，请修改配置文件" << endl;


        if (SynFlag == -1) break; // 文件结束，跳出循环
        else if (SynFlag == 0) 
        { 
            cout <<Raw.RovEpk.Time.Week<<" " << Raw.RovEpk.Time.SecOfWeek << "  " << "时间同步失败" << endl;
            continue;
        } // 同步失败，继续循环
        else ; // 同步成功0

        // 非差周跳探测，流动站SPP,获取卫星位置等中间量        
        DetectOutlier(&Raw.RovEpk);
        MarkValid(&Raw.RovEpk0, &Raw.RovEpk);//根据locktime进行非差观测数据有效性标记
        int f_R = SPP(&Raw.RovEpk, Raw.GpsEph, Raw.BdsEph, &Rres);
        if (f_R) { SPV(&Raw.RovEpk, &Rres); Rres.IsSuccess = true; }
        //cout << Rres.IsSuccess << endl;
        //XYZToBLH(Rres.Pos, Rblh, R_WGS84, F_WGS84);
        //cout << "Rover SPP:  " << Rres.Time.SecOfWeek << "  " << fixed << setprecision(4) << Rres.Pos[0] << "  " << Rres.Pos[1] << "  " << Rres.Pos[2] << "  " 
        //    << fixed << setprecision(8) << "B= " << Rblh[1] << "  " << " L= " << Rblh[0] << "  " << fixed << setprecision(3) << "H= " << Rblh[2] << endl;

        // 非差周跳探测，基站SPP,获取卫星位置等中间量        
        DetectOutlier(&Raw.BasEpk);
        MarkValid(&Raw.BasEpk0, &Raw.BasEpk);
        int f_B = SPP(&Raw.BasEpk, Raw.GpsEph, Raw.BdsEph, &Bres);
        if (f_B) { SPV(&Raw.BasEpk, &Bres); Bres.IsSuccess = true; }
        //cout << Bres.IsSuccess << endl;
        //cout << "Base SPP:  " << Bres.Time.SecOfWeek << "  " << fixed << setprecision(4) << Bres.Pos[0] << "  " << Bres.Pos[1] << "  " << Bres.Pos[2] << "  " << endl;
        //cout << endl;
        
        // 计算站间单差
        FormSDEpochObs(&Raw.RovEpk, &Raw.BasEpk, &Raw.SdObs);//Raw.SdObs.SatNum 16-20颗左右

        // 对单差观测数据进行周跳探测
        DetectCycleSlip(&Raw.SdObs);

        // 确定基准星，可以增加一个对基准星是否解算成功的判断
        DetRefSat(&Raw.RovEpk, &Raw.BasEpk, &Raw.SdObs, &Raw.DDObs);
        /*cout << "历元：" << Raw.SdObs.Time.SecOfWeek << endl;
        for (int a = 0; a < 2; a++)
        {
            if(Raw.DDObs.RefPos[a]!=-1) cout << a << "基准星：" << Raw.DDObs.RefPrn[a] << endl;
            else cout << a << "基准星：未成功找到基准星" << endl;
        }*/

        switch (cfginfo.RTKProMode)
        {
            //EKF
        case 1:
        {
            // 拷贝当前历元双差观测值
            memcpy(&ekf.CurDDObs, &Raw.DDObs, sizeof(DDCOBS));
            // 滤波初始化
            if (ekf.IsInit == false)
            {
                if (!InitFilter(&Raw, &Bres, &Rres, &ekf)) continue;
                // 初始化时需要拷贝当前历元的数据到滤波
                memcpy(&ekf.SDObs, &Raw.SdObs, sizeof(ekf.SDObs));
                memcpy(&ekf.DDObs, &Raw.DDObs, sizeof(ekf.DDObs));
                cout << "历元：" << Raw.SdObs.Time.SecOfWeek << "  初始化  " << endl;
                continue;
            }
            else;

            //时间预测
            if (!EkfPredict(&Raw,&Rres, &ekf)) continue;
            //时间更新待估参数X、P输出
            /*cout << "历元：" << fixed << setprecision(0) << Raw.SdObs.Time.SecOfWeek << "  " << ekf.SDObs.SatNum << "  " << ekf.DDObs.Sats << "  " << Raw.SdObs.SatNum<< "  "<<ekf.CurDDObs.Sats<<"  ";
            for (int a = 0; a < (3 + 2 * ekf.CurDDObs.Sats); a++)
            {
                cout << fixed << setprecision(3) << ekf.X(a) << "  ";
            }
            cout << endl;*/

            //测量更新
            if (!EkfMeasureUpdate(&Raw, &ekf)) continue;

            /*for (int c = 0; c < 2 * ekf.nSats; c++)
            {
                cout << Raw.DDObs.FixedAmb[c] << "  ";
            }
            cout << endl;*/

            //更新前一历元为当前历元
            memcpy(&ekf.SDObs, &Raw.SdObs, sizeof(SDEPOCHOBS));
            memcpy(&ekf.DDObs, &Raw.DDObs, sizeof(DDCOBS));

            break;
        }


        //最小二乘
        case 2:
        {
            //最小二乘浮点解
            if (RTKFloat(&Raw, &Bres, &Rres, &Fres) == false) {  continue; }
            else; // 浮点解解算成功，则进入模糊度固定        
            //cout << "历元：" << Raw.SdObs.Time.SecOfWeek << "  " << fixed << setprecision(8) << Fres.dX[0] << "  " << Fres.dX[1] << "  " << Fres.dX[2] <<"  "<<Raw.DDObs.DDSatNum[0]<< "  " << Raw.DDObs.DDSatNum[1] << endl;


            if (lambda(Raw.DDObs.Sats * 2, 2, Fres.N, Fres.Q, Raw.DDObs.FixedAmb, Raw.DDObs.ResAmb) != 0)
            {
                //cout << "历元：" << Raw.SdObs.Time.SecOfWeek << "模糊度固定失败"; 
                continue;
            }
            else
            {
                //cout<< "历元：" << Raw.SdObs.Time.SecOfWeek << "模糊度固定成功" ;
            }// 模糊度固定成功

            Raw.DDObs.Ratio = Raw.DDObs.ResAmb[1] / Raw.DDObs.ResAmb[0]; // 计算ratio值并标记是否为固定解
            if (Raw.DDObs.Ratio > cfginfo.RatioThres) Raw.DDObs.bFixed = true;
            else
            {
                Raw.DDObs.bFixed = false;
                //cout << "历元：" << Raw.SdObs.Time.SecOfWeek << "Ratio小于3" << "  ratio=" << Raw.DDObs.Ratio << endl; //历元285499有7个小于3，且这一段附近的ratio值较小

            }
            //outfile << Raw.SdObs.Time.SecOfWeek << "  " << Raw.DDObs.Ratio << endl;
            //for (int a = 0; a < 2*Raw.DDObs.Sats; a++)
            //{
            //    outfile << Raw.DDObs.FixedAmb[a] << "  "; //输出模糊度，检查为整数
            //}
            //outfile << endl;
            //for (int b = 2*Raw.DDObs.Sats; b < 4 * Raw.DDObs.Sats; b++)
            //{
            //    outfile << Raw.DDObs.FixedAmb[b] << "  "; //输出模糊度，检查为整数
            //}
            //outfile << endl;

            //最小二乘固定解
            if (RTKFix(&Raw, &Bres, &Rres) == false)
            {
                cout << "历元：" << Raw.SdObs.Time.SecOfWeek << " 最小二乘相对定位固定解失败 "  << "  " << Raw.DDObs.DDSatNum[1] << endl;
            }
            else
            {
                //XYZToBLH(Raw.DDObs.RovPos, Rblh, R_WGS84, F_WGS84);
                //cout << "历元：" << fixed << setprecision(3) << Raw.SdObs.Time.SecOfWeek << "  " << fixed << setprecision(8)
                //    //<<"X= "<< Raw.DDObs.dPos[0] << "  " <<"Y= " << Raw.DDObs.dPos[1] << "  "<<"Z= " << Raw.DDObs.dPos[2] << "  "
                //    << "B= " << (Rblh[1] * Deg) << "  " << " L= " << (Rblh[0] * Deg) << "  " << fixed << setprecision(3) << "H= " << Rblh[2] << "  "
                //    << Raw.DDObs.DDSatNum[0] << "  " << Raw.DDObs.DDSatNum[1] << "  " << Raw.DDObs.Ratio << endl;
            }

            break;
        }


        default: printf("解算方法错误！\n");
        }


        Rres_RTKfloat[0] = Raw.DDObs.dPos_float[0] + Raw.bestpos_base.Pos[0];
        Rres_RTKfloat[1] = Raw.DDObs.dPos_float[1] + Raw.bestpos_base.Pos[1];
        Rres_RTKfloat[2] = Raw.DDObs.dPos_float[2] + Raw.bestpos_base.Pos[2];
        Rres_RTK[0] = ((Raw.DDObs.bFixed == 1) ? Raw.DDObs.dPos[0] : Raw.DDObs.dPos_float[0]) + Raw.bestpos_base.Pos[0];
        Rres_RTK[1] = ((Raw.DDObs.bFixed == 1) ? Raw.DDObs.dPos[1] : Raw.DDObs.dPos_float[1]) + Raw.bestpos_base.Pos[1];
        Rres_RTK[2] = ((Raw.DDObs.bFixed == 1) ? Raw.DDObs.dPos[2] : Raw.DDObs.dPos_float[2]) + Raw.bestpos_base.Pos[2];
        XYZToBLH(Rres_RTKfloat, Rblh_RTKfloat, R_WGS84, F_WGS84);
        XYZToBLH(Rres_RTK, Rblh_RTK, R_WGS84, F_WGS84);
        //结果输出到控制面板
        cout << fixed << setprecision(0) << Raw.SdObs.Time.Week << "  " << fixed << setprecision(3) << Raw.SdObs.Time.SecOfWeek << "  "
            << fixed << setprecision(8) << (Rblh_RTK[1] * Deg) << "  " << (Rblh_RTK[0] * Deg) << "  " << fixed << setprecision(3) << Rblh_RTK[2] << "  "
            << fixed << setprecision(3) << Rres_RTK[0] << "  " << Rres_RTK[1] << "  " << Rres_RTK[2] << "  "
            << fixed << setprecision(3) << Raw.DDObs.dPos[0] << "  " << Raw.DDObs.dPos[1] << "  " << Raw.DDObs.dPos[2] << "  "
            << fixed << setprecision(8) << (Rblh_RTKfloat[1] * Deg) << "  " << (Rblh_RTKfloat[0] * Deg) << "  " << fixed << setprecision(3) << Rblh_RTK[2] << "  "
            << fixed << setprecision(3) << Rres_RTKfloat[0] << "  " << Rres_RTKfloat[1] << "  " << Rres_RTKfloat[2] << "  "
            << Raw.DDObs.bFixed<<"  "<<Raw.DDObs.Sats<<"  "<<Raw.DDObs.DDSatNum[0]<<"  "<<Raw.DDObs.DDSatNum[1]<<"  "
            << fixed << setprecision(5) << Raw.DDObs.PDOP << "  "   //输出精度 RMS和PDOP   << Raw.DDObs.FixRMS[0]
            << fixed << setprecision(3) << Raw.DDObs.Ratio << "  " << endl;

        //结果输出到文件
        outfile << fixed << setprecision(0) << Raw.SdObs.Time.Week << "  " << fixed << setprecision(3) << Raw.SdObs.Time.SecOfWeek << "  "
            << fixed << setprecision(8) << (Rblh_RTK[1] * Deg) << "  " << (Rblh_RTK[0] * Deg) << "  " << fixed << setprecision(3) << Rblh_RTK[2] << "  "
            << fixed << setprecision(12) << Rres_RTK[0] << "  " << Rres_RTK[1] << "  " << Rres_RTK[2] << "  "
            //<< fixed << setprecision(3) << Raw.DDObs.dPos[0] << "  " << Raw.DDObs.dPos[1] << "  " << Raw.DDObs.dPos[2] << "  "
            << fixed << setprecision(8) << (Rblh_RTKfloat[1] * Deg) << "  " << (Rblh_RTKfloat[0] * Deg) << "  " << fixed << setprecision(3) << Rblh_RTK[2] << "  "
            << fixed << setprecision(3) << Rres_RTKfloat[0] << "  " << Rres_RTKfloat[1] << "  " << Rres_RTKfloat[2] << "  "
            << Raw.DDObs.bFixed << "  " << Raw.DDObs.Sats << "  " << Raw.DDObs.DDSatNum[0] << "  " << Raw.DDObs.DDSatNum[1] << "  "
            << fixed << setprecision(5) << Raw.DDObs.PDOP << "  "   //输出精度 RMS和PDOP   << Raw.DDObs.FixRMS[0]
            << fixed << setprecision(3) << Raw.DDObs.Ratio << "  " << endl;

    }

    fclose(frover);
    fclose(fbase);
    outfile.close();
    return 0;
}

