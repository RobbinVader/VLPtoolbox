% 读取全站仪真值数据，2025.11.11
% 2025.11.18更新，使用MS50.FRT格式

function coordinates = read_TS_data(filename, idx1, idx2)
    % 读取坐标数据
    data = readtable(filename, 'Delimiter', ',', 'HeaderLines', 0);
    data.Properties.VariableNames = {'Station', 'X', 'Y', 'Z', 'Time'};
    
    % 筛除data.Station中不是TPS_Auto开头的行
    valid_indices = contains(data.Station, 'TPS_Auto');
    data = data(valid_indices, :);
    
    % 提取点号
    point_numbers = cellfun(@(x) str2double(x{1}), regexp(data.Station, 'TPS_Auto_(\d+)', 'tokens'));

    % 找到指定点号范围内的数据
    selected_indices = point_numbers >= idx1 & point_numbers <= idx2;
    selected_data = data(selected_indices, :);
        
    % 对duration时间进行转换，UTC
    time = seconds(selected_data.Time) - 8*3600;

    % 储存坐标值和时间，交换X和Y
    coordinates = [time, selected_data.Y, selected_data.X, selected_data.Z];

    fprintf('成功读取 %d 行全站仪数据\n', length(time));
end