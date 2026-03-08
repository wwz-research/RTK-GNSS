function PlotRTKSingleFigures(analysisResults, rtkData)
    % PlotRTKSingleFigures 生成独立的RTK关键参数时间序列图（仅显示，不保存）
    % 输入:
    %   analysisResults - 分析结果结构体
    %   rtkData         - RTK原始数据（table 类型）
    
    % ------------------------ 输入参数校验 ------------------------
    if nargin < 2
        error('请传入两个输入参数：analysisResults 和 rtkData！');
    end
    if ~isstruct(analysisResults) || ~istable(rtkData)
        error('参数类型错误：analysisResults需为结构体，rtkData需为table！');
    end

    % ------------------------ 数据预处理 ------------------------
    rtkTable = rtkData;
    n_epochs = height(rtkTable);
    if n_epochs == 0
        error('rtkData为空，无数据可绘图！');
    end

    % GPS时间转换为北京时间小时数（包含小数）
    % GPS纪元起始时间：1980年1月6日00:00:00
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0);
    % 计算每个时刻的GPS datetime
    gps_seconds = rtkTable.GPSWeek * 604800 + rtkTable.GPSWeekSec; % 一周总秒数：7*24*3600=604800
    gps_datetime = gps_epoch + seconds(gps_seconds);
    % 转换为北京时间（UTC+8，GPS时间≈UTC时间，忽略闰秒）
    beijing_datetime = gps_datetime + hours(8);
    % 提取北京时间的小时数（包含分钟和秒的小数部分）
    BeijingTime_Hour = hour(beijing_datetime) + minute(beijing_datetime)/60 + second(beijing_datetime)/3600;

    % dENU（米 → 厘米）
    fixed_dE = analysisResults.fixed_dENU(:,1) * 100;
    fixed_dN = analysisResults.fixed_dENU(:,2) * 100;
    fixed_dH = analysisResults.fixed_dENU(:,3) * 100;
    float_dE = analysisResults.float_dENU(:,1) * 100;
    float_dN = analysisResults.float_dENU(:,2) * 100;
    float_dH = analysisResults.float_dENU(:,3) * 100;

    % Ratio / PDOP / Num / 
    Ratio = rtkTable.Ratio;
    PDOP  = rtkTable.PDOP;
    ratio_mean = analysisResults.statistics.Ratio_mean;
    pdop_mean  = analysisResults.statistics.PDOP_mean;
    SatNum=rtkTable.SatNum;
    GPS_SatNum=rtkTable.GPS_SatNum;
    BDS_SatNum=rtkTable.BDS_SatNum;
    IsFixed=rtkTable.IsFixed;

    
    % ------------------------ 绘图 Start ------------------------
    % (A) 固定解 dENU - 三个方向在同一个figure中用subplot展示
    figure;
    
    % 固定解 dE
    subplot(3,1,1);
    plot(BeijingTime_Hour, fixed_dE, 'r', 'LineWidth', 1.5);
    xlabel('北京时间 (小时)');
    ylabel('Error (cm)');
    title('固定解 dE 时间序列');
    grid minor;
    xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
    
    % 固定解 dN
    subplot(3,1,2);
    plot(BeijingTime_Hour, fixed_dN, 'g', 'LineWidth', 1.5);
    xlabel('北京时间 (小时)');
    ylabel('Error (cm)');
    title('固定解 dN 时间序列');
    grid minor;
    xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
    
    % 固定解 dH
    subplot(3,1,3);
    plot(BeijingTime_Hour, fixed_dH, 'b', 'LineWidth', 1.5);
    xlabel('北京时间 (小时)');
    ylabel('Error (cm)');
    title('固定解 dH 时间序列');
    grid minor;
    xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
    
    fprintf('已显示：固定解 dENU 组合图\n');

    % (B) 浮点解 dENU - 三个方向在同一个figure中用subplot展示
    if any(strcmp({'float_dENU'},'float_dENU'))
        figure;
        
        % 浮点解 dE
        subplot(3,1,1);
        plot(BeijingTime_Hour, float_dE, 'Color', [0, 0.7, 0], 'LineWidth', 1.0);
        xlabel('北京时间 (小时)');
        ylabel('Error (cm)');
        title('浮点解 dE 时间序列');
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        
        % 浮点解 dN
        subplot(3,1,2);
        plot(BeijingTime_Hour, float_dN, 'Color', [0, 0, 0.8], 'LineWidth', 1.0);
        xlabel('北京时间 (小时)');
        ylabel('Error (cm)');
        title('浮点解 dN 时间序列');
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        
        % 浮点解 dH
        subplot(3,1,3);
        plot(BeijingTime_Hour, float_dH, 'Color', [0.8, 0, 0], 'LineWidth', 1.0);
        xlabel('北京时间 (小时)');
        ylabel('Error (cm)');
        title('浮点解 dH 时间序列');
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        
        fprintf('已显示：浮点解 dENU 组合图\n');
    end

     % (C) Ratio
    if any(strcmp({'Ratio'},'Ratio'))
        figure;
        % 处理Ratio数据：大于10的显示为10
        plot_Ratio = min(Ratio, 10);
        plot(BeijingTime_Hour, plot_Ratio, 'Color', [0.5, 0.3, 0.8], 'LineWidth', 1.0); hold on;
        
        % 画Ratio=3阈值线
        hline_3 = plot([BeijingTime_Hour(1), BeijingTime_Hour(end)], [3, 3]);
        set(hline_3, 'LineStyle', '-.', ...
                   'Color', 'r', ...
                   'LineWidth', 1.5);
        
        xlabel('北京时间 (小时)');
        ylabel('Ratio');
        title('Ratio 时间序列（>10显示为10）');
        legend('Ratio (限幅后)','Ratio=3阈值', 'Location','best');
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        ylim([0, 10]); % 设置Y轴范围为0-10，匹配限幅后的数据
        hold off;
        fprintf('已显示：Ratio 图\n');
    end


    % (D) PDOP
    if any(strcmp({'PDOP'},'PDOP'))
        figure;
        plot(BeijingTime_Hour, PDOP, 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0); hold on;
        % 画水平线
        hline = plot([BeijingTime_Hour(1), BeijingTime_Hour(end)], [pdop_mean, pdop_mean]);
        set(hline, 'LineStyle', '--', ...
                   'Color', 'b', ...
                   'LineWidth', 1.5);
        xlabel('北京时间 (小时)');
        ylabel('PDOP');
        title('PDOP 时间序列');
        legend('PDOP', 'PDOP Mean', 'Location','best');
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        hold off;
        fprintf('已显示：PDOP 图\n');
    end

    
    % (E) 卫星数量时序图
    if any(strcmp({'SatNum'},'SatNum'))
        figure;
        plot(BeijingTime_Hour, SatNum, 'b-', 'LineWidth', 1.5); hold on;
        plot(BeijingTime_Hour, GPS_SatNum, 'r--', 'LineWidth', 1.2);
        plot(BeijingTime_Hour, BDS_SatNum, 'g:', 'LineWidth', 1.2);
        
        xlabel('北京时间 (小时)');
        ylabel('Satellite Number');
        title('卫星数量时序变化');
        legend('总卫星数', 'GPS卫星数', 'BDS卫星数', 'Location','best');
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        ylim([0, max(SatNum)+5]); % 设置Y轴范围，留出一定余量
        hold off;
        fprintf('已显示：卫星数量时序图\n');
    end

     % (F) IsFixed状态时序图
    if any(strcmp({'IsFixed'},'IsFixed'))
        figure;
        stairs(BeijingTime_Hour, IsFixed, 'b', 'LineWidth', 1.5); hold on;
        % 添加标记点增强可读性
        plot(BeijingTime_Hour, IsFixed, 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
        
        xlabel('北京时间 (小时)');
        ylabel('解状态');
        title('RTK解状态时序变化');
        ylim([-0.2, 1.2]); % 扩展Y轴范围，避免标签贴边
        yticks([0, 1]);
        yticklabels({'浮点解 (0)', '固定解 (1)'});
        grid minor;
        xlim([min(BeijingTime_Hour), max(BeijingTime_Hour)]);
        hold off;
        fprintf('已显示：RTK解状态时序图\n');
    end

    fprintf('\n所有指定图像已全部显示。\n');
end