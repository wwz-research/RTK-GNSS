% 读取CSV数据文件（第一行为列名）
lsq_data = readtable('dENU_ShortBaseLine_LSQ.csv');
ekf_data = readtable('dENU_ShortBaseLine_EKF.csv');

% ====================== 数据提取与时间转换 ======================
% 最小二乘(LSQ)数据提取（转换为cm）
lsq_gps_week_sec = lsq_data{:, 2};  % GPS周秒
lsq_bj_hour = mod(lsq_gps_week_sec / 3600 + 8, 24);  % 转北京时间（小时）
lsq_dE = lsq_data{:, 3} * 100;  % m -> cm
lsq_dN = lsq_data{:, 4} * 100;
lsq_dU = lsq_data{:, 5} * 100;

% 卡尔曼滤波(EKF)数据提取（转换为cm）
ekf_gps_week_sec = ekf_data{:, 2};
ekf_bj_hour = mod(ekf_gps_week_sec / 3600 + 8, 24);
ekf_dE = ekf_data{:, 3} * 100;  % m -> cm
ekf_dN = ekf_data{:, 4} * 100;
ekf_dU = ekf_data{:, 5} * 100;

% ====================== Figure：dE/dN/dU对比（三子图） ======================
figure('Name', 'dE/dN/dU定位误差对比', 'Position', [100, 100, 800, 600]);

% 子图1：dE方向对比
subplot(3, 1, 1);
plot(lsq_bj_hour, lsq_dE, 'b-', 'LineWidth', 1.2, 'DisplayName', '最小二乘(LSQ)'); hold on;
plot(ekf_bj_hour, ekf_dE, 'r--', 'LineWidth', 1.2, 'DisplayName', '卡尔曼滤波(EKF)');
xlabel('北京时间 (小时)'); ylabel('dE (cm)'); title('dE方向定位误差');
legend('Location', 'best'); grid on; box on;

% 子图2：dN方向对比
subplot(3, 1, 2);
plot(lsq_bj_hour, lsq_dN, 'b-', 'LineWidth', 1.2, 'DisplayName', '最小二乘(LSQ)'); hold on;
plot(ekf_bj_hour, ekf_dN, 'r--', 'LineWidth', 1.2, 'DisplayName', '卡尔曼滤波(EKF)');
xlabel('北京时间 (小时)'); ylabel('dN (cm)'); title('dN方向定位误差');
legend('Location', 'best'); grid on; box on;

% 子图3：dU方向对比
subplot(3, 1, 3);
plot(lsq_bj_hour, lsq_dU, 'b-', 'LineWidth', 1.2, 'DisplayName', '最小二乘(LSQ)'); hold on;
plot(ekf_bj_hour, ekf_dU, 'r--', 'LineWidth', 1.2, 'DisplayName', '卡尔曼滤波(EKF)');
xlabel('北京时间 (小时)'); ylabel('dU (cm)'); title('dU方向定位误差');
legend('Location', 'best'); grid on; box on;