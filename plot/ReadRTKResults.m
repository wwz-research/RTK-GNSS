function rtkData = ReadRTKResults(filename)
    % ReadRTKResults 快速读取零基线RTK定位结果文件
    % 输入:
    %   filePath - 文件路径
    % 输出:
    %   rtkData - 结构体数组，包含所有历元数据，字段与文件列一一对应
    %             字段说明：
    %             - GPSWeek: GPS周
    %             - GPSWeekSec: GPS周秒
    %             - Fixed_B: 固定解纬度（弧度）
    %             - Fixed_L: 固定解经度（弧度）
    %             - Fixed_H: 固定解高程（米）
    %             - Fixed_X: 固定解X坐标（米）
    %             - Fixed_Y: 固定解Y坐标（米）
    %             - Fixed_Z: 固定解Z坐标（米）
    %             - Float_B: 浮点解纬度（弧度）
    %             - Float_L: 浮点解经度（弧度）
    %             - Float_H: 浮点解高程（米）
    %             - Float_X: 浮点解X坐标（米）
    %             - Float_Y: 浮点解Y坐标（米）
    %             - Float_Z: 浮点解Z坐标（米）
    %             - IsFixed: 是否固定解（1=固定，0=不固定）
    %             - SatNum: 总卫星数
    %             - GPS_SatNum: GPS卫星数
    %             - BDS_SatNum: BDS卫星数
    %             - PDOP: PDOP值
    %             - Ratio: Ratio值
    
    % 打开文件
    fileID = fopen(filename, 'r');
    if fileID == -1
        error('无法打开文件');
    end
    
    raw  = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', true, 'HeaderLines', 0);
                        
    % 定义列名
    colNames = {
        'GPSWeek', 'GPSWeekSec', ...
        'Fixed_B', 'Fixed_L', 'Fixed_H', 'Fixed_X', 'Fixed_Y', 'Fixed_Z', ...
        'Float_B', 'Float_L', 'Float_H', 'Float_X', 'Float_Y', 'Float_Z', ...
        'IsFixed', 'SatNum', 'GPS_SatNum', 'BDS_SatNum', 'PDOP', 'Ratio'
        };

    % 构建 table
    rtkData = table( ...
        raw{1}, raw{2}, raw{3}, raw{4}, raw{5}, raw{6}, raw{7}, raw{8}, ...
        raw{9}, raw{10}, raw{11}, raw{12}, raw{13}, raw{14}, ...
        raw{15}, raw{16}, raw{17}, raw{18}, raw{19}, raw{20}, ...
        'VariableNames', colNames ...
        );
    
end