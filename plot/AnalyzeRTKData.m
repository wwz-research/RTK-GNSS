function analysisResults = AnalyzeRTKData(rtkData, outputPath)
    % AnalyzeRTKData 分析RTK数据，计算统计量、dENU、历元级RMSE及整体RMSE（含CSV导出）
    % 输入:
    %   rtkData   - RTK数据（table类型或struct数组，来自ReadRTKResults函数）
    %   outputPath- 可选，CSV文件输出路径（默认：'dENU_Results.csv'）
    % 输出:
    %   analysisResults - 结构体，包含所有分析结果
    
    % ------------------------ 参数初始化 ------------------------
    if nargin < 2
        outputPath = 'dENU_Results.csv';  % 默认输出文件名
    end
    
    % ------------------------ 数据预处理 ------------------------
    rtkTable = rtkData;
    
    % 提取数据列（新增GPS时间字段）
    fixed_X = rtkTable.Fixed_X;
    fixed_Y = rtkTable.Fixed_Y;
    fixed_Z = rtkTable.Fixed_Z;
    float_X = rtkTable.Float_X;
    float_Y = rtkTable.Float_Y;
    float_Z = rtkTable.Float_Z;
    PDOP = rtkTable.PDOP;
    Ratio = rtkTable.Ratio;
    SatNum = rtkTable.SatNum;
    isFixed = rtkTable.IsFixed;
    GPSWeek = rtkTable.GPSWeek;      
    GPSWeekSec = rtkTable.GPSWeekSec; 
    n_epochs = height(rtkTable);
    
    % ------------------------ 计算统计量（均值+标准差） ------------------------
    analysisResults.statistics = struct();
    % 固定解XYZ统计
    analysisResults.statistics.fixed_X_mean = mean(fixed_X);
    analysisResults.statistics.fixed_Y_mean = mean(fixed_Y);
    analysisResults.statistics.fixed_Z_mean = mean(fixed_Z);
    analysisResults.statistics.fixed_X_std = std(fixed_X);
    analysisResults.statistics.fixed_Y_std = std(fixed_Y);
    analysisResults.statistics.fixed_Z_std = std(fixed_Z);
    
    % 浮点解XYZ统计
    analysisResults.statistics.float_X_mean = mean(float_X);
    analysisResults.statistics.float_Y_mean = mean(float_Y);
    analysisResults.statistics.float_Z_mean = mean(float_Z);
    analysisResults.statistics.float_X_std = std(float_X);
    analysisResults.statistics.float_Y_std = std(float_Y);
    analysisResults.statistics.float_Z_std = std(float_Z);
    
    % PDOP和Ratio统计
    analysisResults.statistics.PDOP_mean = mean(PDOP);
    analysisResults.statistics.PDOP_std = std(PDOP);
    analysisResults.statistics.SatNum_mean = mean(SatNum);
    analysisResults.statistics.SatNum_std = std(SatNum);
    analysisResults.statistics.Ratio_mean = mean(Ratio);
    analysisResults.statistics.Ratio_std = std(Ratio);
    analysisResults.statistics.fixed_epochs_ratio = sum(isFixed)/n_epochs;
    
    % 参考坐标
    % analysisResults.ref_coords = [analysisResults.statistics.fixed_X_mean, ...
    %                              analysisResults.statistics.fixed_Y_mean, ...
    %                              analysisResults.statistics.fixed_Z_mean];
    % 22零基线事后RTK定位参考坐标（采用老师提供的参考文件固定解平均值作为精确坐标进行分析）
    % analysisResults.ref_coords = [ -2267799.408476, 5009331.073115, 3220984.552516];
    % 25d短基线事后RTK定位参考坐标（采用老师提供的参考文件固定解平均值作为精确坐标进行分析）
    analysisResults.ref_coords = [ -2267808.336856440175, 5009321.489190992899, 3221021.847353241406];
    
    % ------------------------ 计算dENU ------------------------
    R = 6378137;          % WGS84地球半长轴
    F = 1/298.257223563;  % WGS84地球扁率
    
    analysisResults.fixed_dENU = zeros(n_epochs, 3);
    analysisResults.float_dENU = zeros(n_epochs, 3);
    
    for i = 1:n_epochs
        fixed_coord = [fixed_X(i), fixed_Y(i), fixed_Z(i)];
        analysisResults.fixed_dENU(i,:) = CoordinateTransformation(analysisResults.ref_coords, fixed_coord, R, F);
        
        float_coord = [float_X(i), float_Y(i), float_Z(i)];
        analysisResults.float_dENU(i,:) = CoordinateTransformation(analysisResults.ref_coords, float_coord, R, F);
    end
    
    
    % ------------------------ 导出数据到CSV ------------------------
    csvData = table(...
        GPSWeek, ...                  % GPS周
        GPSWeekSec, ...               % GPS周秒
        analysisResults.fixed_dENU(:,1), ...
        analysisResults.fixed_dENU(:,2), ...
        analysisResults.fixed_dENU(:,3), ...
        analysisResults.float_dENU(:,1), ...
        analysisResults.float_dENU(:,2), ...
        analysisResults.float_dENU(:,3), ...
        PDOP, ... 
        'VariableNames', { ...
            'GPS_Week', ...           % CSV列名：GPS周
            'GPS_Week_Sec', ...       % CSV列名：GPS周秒
            'Fixed_dE_m', ...
            'Fixed_dN_m', ...
            'Fixed_dH_m', ...
            'Float_dE_m', ...
            'Float_dN_m', ...
            'Float_dH_m', ...
            'PDOP', ... 
        });
    
    % 写入CSV文件（保持原数据类型，GPS周为整数，周秒为浮点数）
    writetable(csvData, outputPath, 'WriteVariableNames', true);
    fprintf('GPS时间、dENU及历元RMSE数据已导出到CSV文件：%s\n', fullfile(pwd, outputPath));

    % ------------------------ 计算整体RMSE ------------------------
    analysisResults.fixed_RMSE = sqrt(mean(analysisResults.fixed_dENU.^2, 1));  % [1 x 3]
    analysisResults.float_RMSE = sqrt(mean(analysisResults.float_dENU.^2, 1));  % [1 x 3]
    
    % ------------------------ dENU统计信息 ------------------------
    analysisResults.fixed_dENU_stats = struct(...
        'mean', mean(analysisResults.fixed_dENU, 1), ...
        'std', std(analysisResults.fixed_dENU, [], 1), ...
        'max', max(analysisResults.fixed_dENU, [], 1), ...
        'min', min(analysisResults.fixed_dENU, [], 1));
    
    analysisResults.float_dENU_stats = struct(...
        'mean', mean(analysisResults.float_dENU, 1), ...
        'std', std(analysisResults.float_dENU, [], 1), ...
        'max', max(analysisResults.float_dENU, [], 1), ...
        'min', min(analysisResults.float_dENU, [], 1));
    
    % ------------------------ 输出统计信息 ------------------------
    fprintf('================ RTK数据分析结果 ================\n');
    fprintf('参考坐标（固定解均值）：\n');
    fprintf('  X: %.6f m, Y: %.6f m, Z: %.6f m\n', analysisResults.ref_coords);
    fprintf('\n坐标统计（均值±标准差）：\n');
    fprintf('  固定解XYZ：%.6f±%.6f, %.6f±%.6f, %.6f±%.6f m\n', ...
        analysisResults.statistics.fixed_X_mean, analysisResults.statistics.fixed_X_std, ...
        analysisResults.statistics.fixed_Y_mean, analysisResults.statistics.fixed_Y_std, ...
        analysisResults.statistics.fixed_Z_mean, analysisResults.statistics.fixed_Z_std);
    fprintf('  浮点解XYZ：%.6f±%.6f, %.6f±%.6f, %.6f±%.6f m\n', ...
        analysisResults.statistics.float_X_mean, analysisResults.statistics.float_X_std, ...
        analysisResults.statistics.float_Y_mean, analysisResults.statistics.float_Y_std, ...
        analysisResults.statistics.float_Z_mean, analysisResults.statistics.float_Z_std);
    fprintf('\nPDOP & Ratio统计（均值±标准差）：\n');
    fprintf('  PDOP：%.3f±%.3f,SatNum：%.3f±%.3f,Ratio：%.3f±%.3f\n', ...
        analysisResults.statistics.PDOP_mean, analysisResults.statistics.PDOP_std, ...
        analysisResults.statistics.SatNum_mean, analysisResults.statistics.SatNum_std, ...
        analysisResults.statistics.Ratio_mean, analysisResults.statistics.Ratio_std);
    fprintf('  固定解历元占比：%.1f%%\n', ...
        analysisResults.statistics.fixed_epochs_ratio*100);
    fprintf('\n各分量全局RMSE值（dE, dN, dH）：\n');
    fprintf('  固定解：%.3f, %.3f, %.3f cm\n', ...
        analysisResults.fixed_RMSE*100);
    fprintf('  浮点解：%.3f, %.3f, %.3f cm\n', ...
        analysisResults.float_RMSE*100);
    fprintf('================================================\n');
    fprintf('\n固定解dENU详细统计（单位：cm）：\n');
    fprintf('        dE          dN          dH\n');
    fprintf('均值   %.10f       %.10f       %.10f\n', analysisResults.fixed_dENU_stats.mean*100);
    fprintf('标准差 %.3f       %.3f       %.3f\n', analysisResults.fixed_dENU_stats.std*100);
    fprintf('最大值 %.3f       %.3f       %.3f\n', analysisResults.fixed_dENU_stats.max*100);
    fprintf('最小值 %.3f       %.3f       %.3f\n', analysisResults.fixed_dENU_stats.min*100);
    
    fprintf('\n浮点解dENU详细统计（单位：cm）：\n');
    fprintf('        dE          dN          dH\n');
    fprintf('均值   %.3f       %.3f       %.3f\n', analysisResults.float_dENU_stats.mean*100);
    fprintf('标准差 %.3f       %.3f       %.3f\n', analysisResults.float_dENU_stats.std*100);
    fprintf('最大值 %.3f       %.3f       %.3f\n', analysisResults.float_dENU_stats.max*100);
    fprintf('最小值 %.3f       %.3f       %.3f\n', analysisResults.float_dENU_stats.min*100);
end