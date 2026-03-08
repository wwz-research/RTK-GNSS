#include "SPP.h"


/****************************************************************************
  XYZToBLH

  目的：定义笛卡尔坐标到大地坐标的函数

  输入参数：
    xyz[0] = X, xyz[1] = Y, xyz[2] = Z
    R = 地球半长轴，F = 地球扁率
  输出参数：
    blh[0] = 经度L（弧度），blh[1] = 纬度B（弧度），blh[2] = 高度H

****************************************************************************/
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F) 
{
    // 计算偏心率
    double e2 = 2 * F - F * F;

    // 初始化ΔZ
    double deltaZ = e2 * xyz[2];

    // 迭代参数
    const int maxIterations = 100; // 最大迭代次数
    const double epsilon = 1e-12;  // 精度阈值
    int iterationCount = 0;        // 迭代计数器

    // 迭代计算ΔZ，直到满足精度要求或达到最大迭代次数
    double prevDeltaZ = 0;
    //bool converged = false;
    while (std::abs(deltaZ - prevDeltaZ) > epsilon && iterationCount < maxIterations) {
        prevDeltaZ = deltaZ;

        // 计算纬度B
        double sqrtXY = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
        double B = std::atan((xyz[2] + deltaZ)/ sqrtXY);

        // 计算sin(B)
        double sinB = std::sin(B);

        // 计算N
        double N = R / std::sqrt(1 - e2 * sinB * sinB);

        // 更新ΔZ
        deltaZ = N * e2 * sinB;

        iterationCount++;
    }

    if (iterationCount >= maxIterations) {
        std::cerr << "Warning: Iteration did not converge within " << maxIterations << " iterations." << std::endl;
    }

    // 计算经度L
    blh[0] = std::atan2(xyz[1], xyz[0]);

    // 计算纬度B
    double sqrtXY = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
    blh[1] = std::atan((xyz[2] + deltaZ)/ sqrtXY);

    // 计算高度H
    double N = R / std::sqrt(1 - e2 * std::sin(blh[1]) * std::sin(blh[1]));
    blh[2] = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + (xyz[2] + deltaZ) * (xyz[2] + deltaZ)) - N;

}

/****************************************************************************
  BLHToXYZ

  目的：定义大地坐标到笛卡尔坐标的函数

  输入参数：
    BLH[0] = 经度L（弧度），BLH[1] = 纬度B（弧度），BLH[2] = 高度H（米）
    R = 地球半长轴，F = 地球扁率
  输出参数：
    XYZ[0] = X，XYZ[1] = Y，XYZ[2] = Z

****************************************************************************/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F) {
    
  
    // 计算偏心率
    double e2 = 2 * F - F * F;

    // 计算卯酉圈半径N
    double N = R / std::sqrt(1 - e2 * sin(BLH[1])* sin(BLH[1]));

    // 计算X、Y、Z坐标
    XYZ[0] = (N + BLH[2]) * cos(BLH[1]) * cos(BLH[0]);
    XYZ[1] = (N + BLH[2]) * cos(BLH[1]) * sin(BLH[0]);
    XYZ[2] = (N * (1 - e2) + BLH[2]) * sin(BLH[1]);
}

/****************************************************************************
  BLHToNEUMat

  目的：测站地平坐标转换矩阵计算函数

  输入参数：
     Blh[0] = 经度L（弧度），Blh[1] = 纬度B（弧度）
  输出参数：
     Mat 是一个3x3的旋转矩阵

****************************************************************************/
void BLHToNEUMat(const double blh[], double Mat[3][3]) {
   
    // 构建旋转矩阵
    Mat[0][0] = -sin(blh[0]);
    Mat[0][1] = cos(blh[0]);
    Mat[0][2] = 0.0;

    Mat[1][0] = -sin(blh[1]) * cos(blh[0]);
    Mat[1][1] = -sin(blh[1]) * sin(blh[0]);
    Mat[1][2] = cos(blh[1]);

    Mat[2][0] = cos(blh[1]) * cos(blh[0]);
    Mat[2][1] = cos(blh[1]) * sin(blh[0]);
    Mat[2][2] = sin(blh[1]);
}

/****************************************************************************
  CompSatElAz

  目的：卫星高度角方位角计算函数

  输入参数：
   Xr 是测站的XYZ坐标,Xs 是卫星的XYZ坐标
   R = 地球半长轴，F = 地球扁率
  输出参数：
   Elev 是卫星的高度角（弧度）
   Azim 是卫星的方位角（弧度）

***************************************************************************/
void CompSatElAz(const double Xr[],const double Xs[], const double R, const double F, double* Elev, double* Azim) {
    // 计算测站到卫星的向量
    Eigen::Vector3d dx(Xs[0] - Xr[0], Xs[1] - Xr[1], Xs[2] - Xr[2]);

    // 创建旋转矩阵
    double Mat[3][3];
    double blh[3];
    XYZToBLH(Xr, blh, R, F);
    BLHToNEUMat(blh, Mat);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotationMat(&Mat[0][0]);// 显式指定行优先存储 (Eigen::RowMajor)
    //Eigen::Map<Eigen::Matrix3d> rotationMat(&Mat[0][0]);// 将 C 风格数组转换为 Eigen 矩阵
    //使用 Eigen::Map 时，它不会复制数据，而是直接将内存中的数据重新解释为 Eigen 的矩阵或向量

    // 将测站到卫星的向量转换到地平坐标系
    Eigen::Vector3d dEnu = rotationMat * dx;

    // 计算高度角和方位角
    *Elev = std::atan(dEnu[2]/ std::sqrt(dEnu[0] * dEnu[0] + dEnu[1] * dEnu[1]));
    *Azim = std::atan2(dEnu[0], dEnu[1]);
}


/****************************************************************************
  Comp_dEnu

  目的：定位误差计算函数

  输入参数：
    X0 是测站的精确XYZ坐标 Xr 是测站的估计XYZ坐标
    R = 地球半长轴，F = 地球扁率
  输出参数：
    dNeu 是地平坐标系下的定位误差（dE, dN, dH）

****************************************************************************/
void Comp_dEnu(const double X0[], const double Xr[], const double R, const double F, double dNeu[]) {
    // 创建 C 风格数组来存储旋转矩阵
    double Mat[3][3];
    double blh[3];
    XYZToBLH(Xr, blh, R, F);
    BLHToNEUMat(blh, Mat);
    Eigen::Map<Eigen::Matrix3d> rotationMat(&Mat[0][0]);

    // 计算坐标差
    Eigen::Vector3d dx(Xr[0] - X0[0], Xr[1] - X0[1], Xr[2] - X0[2]);

    // 将坐标差转换到地平坐标系
    Eigen::Vector3d dEnu = rotationMat * dx;

    // 将结果复制到输出数组
    std::copy(dEnu.data(), dEnu.data() + 3, dNeu);
}