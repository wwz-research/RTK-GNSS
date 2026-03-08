% ------------------------ 初始化设置 ------------------------
clc; clearvars; close all;  % 清除命令行、工作区变量、关闭旧图像
fprintf('============ RTK数据处理流程开始 ============\n\n');

% ------------------------ 用户参数配置 ------------------------
dataFilePath = 'ShortBaseLine_EKF_noise5.txt';  % RTK原始数据文件路径           RealTime_EKF.txt           ShortBaseLine_EKF.txt
csvOutputPath = 'dENU_ShortBaseLine_EKF_noise5.csv'; % 分析结果CSV输出路径      dENU_RealTime_EKF.csv      dENU_ShortBaseLine_EKF.csv
plotTypes = {'fixed_dENU', 'float_dENU', 'Ratio', 'PDOP', 'SatNum', 'IsFixed'};  % 要绘制的图像类型

% ------------------------ 步骤1：读取RTK数据 ------------------------
fprintf('步骤1：读取RTK原始数据...\n');
try
    rtkData = ReadRTKResults(dataFilePath);  % 调用数据读取函数
    fprintf('✅ 文件读取成功！共读取 %d 个历元\n\n数据', height(rtkData));
catch ME
    fprintf('❌ 文件读取失败：%s\n', ME.message);
    fprintf('请检查文件路径是否正确，或文件格式是否匹配！\n');
    return;  % 读取失败则终止流程
end

% ------------------------ 步骤2：分析RTK数据 ------------------------
fprintf('步骤2：分析RTK数据（计算dENU、RMS、统计量）...\n');
try
    analysisResults = AnalyzeRTKData(rtkData, csvOutputPath);  % 调用分析函数
    fprintf('✅ 数据解析完成！分析结果已保存至CSV文件：%s\n\n', csvOutputPath);
catch ME
    fprintf('❌ 数据解析失败：%s\n', ME.message);
    return;
end

% ------------------------ 步骤3：绘制时间序列图像 ------------------------
fprintf('步骤3：绘制时间序列图像...\n');
try
    PlotRTKSingleFigures(analysisResults,rtkData);        
    fprintf('✅ 图像绘制完成！所有图像已显示\n\n');
catch ME
    fprintf('❌ 图像绘制失败：%s\n', ME.message);
    return;
end

% ------------------------ 流程结束 ------------------------
fprintf('============ RTK数据处理流程完成 ============\n');
fprintf('所有步骤执行完毕，结果如下：\n');
fprintf('  - 原始数据：%s\n', dataFilePath);
fprintf('  - 分析结果：%s\n', csvOutputPath);
fprintf('  - 图像类型：%s\n', strjoin(plotTypes, ', '));