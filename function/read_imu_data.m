function imu_data = read_imu_data(filename)
% 读取xsens经MT Manager转化得到的惯导数据文件
% 输入参数：
%   filename - 数据文件名
% 输出参数：
%   imu_data - 包含所有数据的结构体
% 使用示例
% 将您的数据保存为.txt文件，然后运行：
% imu_data = read_imu_data('your_imu_data.txt');

    % 读取文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('无法打开文件: %s', filename);
    end
    
    % 跳过文件头信息（直到找到数据列标题行）
    header_lines = 0;
    while true
        line = fgetl(fid);
        header_lines = header_lines + 1;
        if contains(line, 'PacketCounter') || contains(line, 'PacketCounter')
            break;
        end
        if ~ischar(line)
            error('文件格式错误：未找到数据列标题');
        end
    end
    
    % 关闭文件，用textscan重新读取
    fclose(fid);
    
    % 定义数据格式
    formatSpec = '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
    
    % 读取数据
    fid = fopen(filename, 'r');
    % 跳过文件头
    for i = 1:header_lines
        fgetl(fid);
    end
    
    % 读取数据
    data = textscan(fid, formatSpec);
    fclose(fid);
    
    % 将数据存储到结构体中
    imu_data.PacketCounter = data{1};
    imu_data.SampleTimeFine = data{2};
    imu_data.Year = data{3};
    imu_data.Month = data{4};
    imu_data.Day = data{5};
    imu_data.Second = data{6};
    imu_data.Acc_X = data{7};  % 加速度 X (m/s²)
    imu_data.Acc_Y = data{8};  % 加速度 Y (m/s²)
    imu_data.Acc_Z = data{9};  % 加速度 Z (m/s²)
    imu_data.Gyr_X = data{10}; % 角速度 X (rad/s)
    imu_data.Gyr_Y = data{11}; % 角速度 Y (rad/s)
    imu_data.Gyr_Z = data{12}; % 角速度 Z (rad/s)
    imu_data.Roll = data{13};  % 横滚角 (度)
    imu_data.Pitch = data{14}; % 俯仰角 (度)
    imu_data.Yaw = data{15};   % 偏航角 (度)
    
    % 计算时间序列（以秒为单位）
    % 假设SampleTimeFine是微秒级时间戳
    imu_data.Time = (imu_data.SampleTimeFine - imu_data.SampleTimeFine(1)) * 1e-4;
    
    % 显示数据基本信息
    fprintf('成功读取 %d 行IMU数据，', length(imu_data.PacketCounter));
    fprintf('数据时间范围: %.3f 秒，', imu_data.Time(end));
    fprintf('采样频率: %.2f Hz\n', length(imu_data.Time)/imu_data.Time(end));
end
