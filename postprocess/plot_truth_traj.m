% plot_truth_traj.m
% 绘制当前配置数据对应的全站仪真值轨迹。
% 数据读取、时间同步、仪器高和杆臂校正流程参考 VLP_3D_atti.m。

clear;
close all;
clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(script_dir);
addpath(fullfile(repo_root, 'function'));

cfg_file = fullfile(repo_root, 'config', '20251127_6.yaml');
[~, TSfile, IMUfile, idx1, idx2, t0_TS, t0_IMU, ~, yaw0] = ...
    config_from_yaml(cfg_file);
[LED, nLED] = VLP_parameter(fullfile(repo_root, 'config', 'VLP_parameter.yaml'), 3);

TS_h = 1.5963;
leverarm = [0.11; 0.10; -0.02];

imu_data = read_imu_data(fullfile(repo_root, 'data', IMUfile));
imu_data.Time = imu_data.Time + t0_IMU;
imu_data.Yaw = imu_data.Yaw + 180;
imu_data.Yaw = imu_data.Yaw - imu_data.Yaw(1) + yaw0;

coordinates = read_TS_data(fullfile(repo_root, 'data', TSfile), idx1, idx2);
coordinates(:, 1) = coordinates(:, 1) - coordinates(1, 1) + t0_TS;
coordinates(:, 4) = coordinates(:, 4) + TS_h;
coordinates = leverarm_corr_TS(coordinates, leverarm, imu_data);

fprintf('真值轨迹点数: %d\n', size(coordinates, 1));
fprintf('时间范围: %.3f s - %.3f s\n', coordinates(1, 1), coordinates(end, 1));

figure('Name', '全站仪真值轨迹', 'Color', 'w');
plot3(coordinates(:, 2), coordinates(:, 3), coordinates(:, 4), ...
    'b-', 'LineWidth', 1.5, 'DisplayName', '真值轨迹');
hold on;
plot3(coordinates(1, 2), coordinates(1, 3), coordinates(1, 4), ...
    'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot3(coordinates(end, 2), coordinates(end, 3), coordinates(end, 4), ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
scatter3(LED(:, 1), LED(:, 2), LED(:, 3), 70, 'r*', 'DisplayName', 'LED');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('全站仪真值三维轨迹');
legend('Location', 'best');

figure('Name', '真值轨迹平面图', 'Color', 'w');
plot(coordinates(:, 2), coordinates(:, 3), ...
    'b-', 'LineWidth', 1.5, 'DisplayName', '真值轨迹');
hold on;
plot(coordinates(1, 2), coordinates(1, 3), ...
    'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(coordinates(end, 2), coordinates(end, 3), ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', '终点');
scatter(LED(:, 1), LED(:, 2), 70, 'r*', 'DisplayName', 'LED');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
title('全站仪真值 XY 平面轨迹');
legend('Location', 'best');

figure('Name', '真值坐标时间序列', 'Color', 'w');
component_names = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(3, 1, i);
    plot(coordinates(:, 1), coordinates(:, i + 1), ...
        'LineWidth', 1.2, 'DisplayName', component_names{i});
    grid on;
    ylabel([component_names{i}, ' (m)']);
    legend('Location', 'best');
end
xlabel('Time (s)');
sgtitle('全站仪真值坐标随时间变化');

fprintf('真值轨迹绘制完成，配置文件: %s\n', cfg_file);
