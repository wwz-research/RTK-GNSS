function dNeu = CoordinateTransformation(X0, Xr, R, F)
    % CoordinateTransformation 计算地平坐标系下的定位误差(dE, dN, dH)
    % 功能整合：XYZ转BLH、BLH转NEU旋转矩阵、定位误差计算
    % 输入:
    %   X0 - 1x3向量，测站精确笛卡尔坐标 [X0, Y0, Z0] (米)
    %   Xr - 1x3向量，测站估计笛卡尔坐标 [Xr, Yr, Zr] (米)
    %   R  - 地球半长轴 (米，如WGS84的6378137)
    %   F  - 地球扁率 (如WGS84的1/298.257223563)
    % 输出:
    %   dNeu - 1x3向量，地平坐标系误差 [dE, dN, dH] (米)
    
    % 步骤1：将估计坐标Xr转换为大地坐标BLH
    blh = XYZToBLH(Xr, R, F);
    
    % 步骤2：根据BLH计算NEU旋转矩阵
    neuMat = BLHToNEUMat(blh);
    
    % 步骤3：计算笛卡尔坐标差并转换到NEU坐标系
    deltaXYZ = Xr - X0;  % 估计坐标与精确坐标的差值
    dNeu = (neuMat * deltaXYZ')';  % 矩阵乘法需列向量，结果转置为行向量
    
end

% ------------------------------ 子函数1：XYZ转BLH ------------------------------
function blh = XYZToBLH(xyz, R, F)
    % XYZToBLH 笛卡尔坐标(XYZ)转大地坐标(BLH)
    e2 = 2*F - F^2;  % 第一偏心率平方
    deltaZ = e2 * xyz(3);
    
    % 迭代求解大地纬度（牛顿迭代法）
    maxIter = 100;
    tol = 1e-12;
    iter = 0;
    prevDeltaZ = 0;
    
    while abs(deltaZ - prevDeltaZ) > tol && iter < maxIter
        prevDeltaZ = deltaZ;
        sqrtXY = sqrt(xyz(1)^2 + xyz(2)^2);
        B = atan((xyz(3)+deltaZ)/sqrtXY);  % 纬度近似值
        sinB = sin(B);
        N = R / sqrt(1 - e2*sinB^2);       % 卯酉圈曲率半径
        deltaZ = N * e2 * sinB;            % 更新ΔZ
        iter = iter + 1;
    end
    
    if iter >= maxIter
        warning('XYZToBLH迭代未收敛，已达最大迭代次数！');
    end
    
    % 计算最终BLH
    L = atan2(xyz(2), xyz(1));  % 经度（弧度）
    sqrtXY = sqrt(xyz(1)^2 + xyz(2)^2);
    B = atan((xyz(3)+deltaZ)/sqrtXY);  % 纬度（弧度）
    N = R / sqrt(1 - e2*sin(B)^2);
    H = sqrt(sqrtXY^2 + (xyz(3)+deltaZ)^2) - N;  % 大地高度
    
    blh = [L, B, H];
end

% ------------------------------ 子函数2：BLH转NEU矩阵 ------------------------------
function mat = BLHToNEUMat(blh)
    % BLHToNEUMat 大地坐标(BLH)转NEU地平坐标系旋转矩阵
    L = blh(1);  % 经度
    B = blh(2);  % 纬度
    
    mat = zeros(3,3);
    % 东向（E）分量
    mat(1,1) = -sin(L);
    mat(1,2) = cos(L);
    mat(1,3) = 0;
    % 北向（N）分量
    mat(2,1) = -sin(B)*cos(L);
    mat(2,2) = -sin(B)*sin(L);
    mat(2,3) = cos(B);
    % 天向（U/H）分量
    mat(3,1) = cos(B)*cos(L);
    mat(3,2) = cos(B)*sin(L);
    mat(3,3) = sin(B);
end