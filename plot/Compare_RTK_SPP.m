%% ============================================================
%  绘制 SPP、RTK Float、RTK Fixed 的 dENU 对比时序图（以北京时间小时为横轴）
% =============================================================

clear; clc; close all;

%% -------------------- 读入 CSV 文件 --------------------
file_spp  = "dENU_ShortBaseLine_SPP.csv";
file_rtk  = "dENU_ShortBaseLine_EKF.csv";

data_spp = readtable(file_spp);
data_rtk = readtable(file_rtk);

% 文件列结构：
% SPP: week | sow | dE | dN | dU  （单位：m）
% RTK: week | sow | dE_fix | dN_fix | dU_fix | dE_flt | dN_flt | dU_flt （单位：m）

%% -------------------- 转换为北京时间（小时） --------------------
GPS_WEEK_SPP = data_spp{:,1};
SOW_SPP      = data_spp{:,2};

GPS_WEEK_RTK = data_rtk{:,1};
SOW_RTK      = data_rtk{:,2};

% GPS 时间 → UTC（GPS 比 UTC 快 18 秒）
UTC_spp = SOW_SPP - 18;
UTC_rtk = SOW_RTK - 18;

% UTC → 北京时间 (UTC+8 小时)
BJ_hour_spp = (UTC_spp ./ 3600) + 8;
BJ_hour_rtk = (UTC_rtk ./ 3600) + 8;

% 仅保留小时小数形式（不需要日期信息）
BJ_hour_spp = mod(BJ_hour_spp, 24);
BJ_hour_rtk = mod(BJ_hour_rtk, 24);

%% -------------------- 取出 dENU（转换为 cm） --------------------
dE_spp = data_spp{:,3} * 100;
dN_spp = data_spp{:,4} * 100;
dU_spp = data_spp{:,5} * 100;

dE_fix = data_rtk{:,3} * 100;
dN_fix = data_rtk{:,4} * 100;
dU_fix = data_rtk{:,5} * 100;

dE_flt = data_rtk{:,6} * 100;
dN_flt = data_rtk{:,7} * 100;
dU_flt = data_rtk{:,8} * 100;

PDOP_spp = data_spp{:,9};
PDOP_rtk = data_rtk{:,9};

%% ============================================================
%  绘图（北京时序）
% ============================================================

figure('Name','SPP / RTK Float / RTK Fixed dENU Comparison');

titles = {'E方向定位误差 (cm)','N方向定位误差 (cm)','U方向定位误差 (cm)'};

% 曲线颜色设定
color_spp = [0 0.447 0.741];    % 蓝
color_flt = [0.85 0.325 0.098]; % 红
color_fix = [0.466 0.674 0.188];% 绿

% ========== E方向 ==========
subplot(3,1,1);
hold on; grid on;
plot(BJ_hour_spp, dE_spp, '.', 'Color', color_spp, 'MarkerSize', 6);
plot(BJ_hour_rtk, dE_flt, '.', 'Color', color_flt, 'MarkerSize', 6);
plot(BJ_hour_rtk, dE_fix, '.', 'Color', color_fix, 'MarkerSize', 6);
ylabel('dE (cm)');
title(titles{1});
legend('SPP','RTK Float','RTK Fixed');

% ========== N方向 ==========
subplot(3,1,2);
hold on; grid on;
plot(BJ_hour_spp, dN_spp, '.', 'Color', color_spp, 'MarkerSize', 6);
plot(BJ_hour_rtk, dN_flt, '.', 'Color', color_flt, 'MarkerSize', 6);
plot(BJ_hour_rtk, dN_fix, '.', 'Color', color_fix, 'MarkerSize', 6);
ylabel('dN (cm)');
title(titles{2});
legend('SPP','RTK Float','RTK Fixed');

% ========== U方向 ==========
subplot(3,1,3);
hold on; grid on;
plot(BJ_hour_spp, dU_spp, '.', 'Color', color_spp, 'MarkerSize', 6);
plot(BJ_hour_rtk, dU_flt, '.', 'Color', color_flt, 'MarkerSize', 6);
plot(BJ_hour_rtk, dU_fix, '.', 'Color', color_fix, 'MarkerSize', 6);
ylabel('dU (cm)');
xlabel('北京时间（小时）');
title(titles{3});
legend('SPP','RTK Float','RTK Fixed');

%% ============================================================
%  PDOP 时间序列对比图（SPP vs RTK）
% ============================================================

figure('Name','SPP / RTK PDOP Time Series');
hold on; grid on;

plot(BJ_hour_spp, PDOP_spp, '-', 'Color', color_spp, 'MarkerSize', 8);
plot(BJ_hour_rtk, PDOP_rtk, '--', 'Color', color_fix, 'MarkerSize', 8);

xlabel('北京时间（小时）');
ylabel('PDOP');
title('PDOP 时间序列对比（SPP vs RTK）');
legend('SPP','RTK');