% VLP/INS loose coupling postprocess with NED visualization.
% The positioning pipeline follows VLP_3D_atti.m, while plots and
% statistics are organized here without modifying the original script.

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fullfile(script_dir, '..');
addpath(fullfile(repo_root, 'function'));

config_file = fullfile(repo_root, 'config', '20251127_6.yaml');
vlp_param_file = fullfile(repo_root, 'config', 'VLP_parameter.yaml');
data_dir = fullfile(repo_root, 'data');

[VLPfile, TSfile, IMUfile, idx1, idx2, t0_TS, t0_IMU, std, yaw0] = ...
    config_from_yaml(config_file);
[LED, nLED, a, M, fhz, fs, dt, rate] = ...
    VLP_parameter(vlp_param_file, 3);

TS_h = 1.5963;
leverarm = [0.11; 0.10; -0.02];
x0 = 4.5;
y0 = 1.5;
h = 0;

if strcmp(VLPfile, 'newest')
    VLPfile = read_newest_VLP(data_dir);
end

fid3 = load(fullfile(data_dir, VLPfile));
fid3 = fid3(2:end-1);

imu_data = read_imu_data(fullfile(data_dir, IMUfile));
imu_data.Time = imu_data.Time + t0_IMU;
imu_data.Yaw = imu_data.Yaw + 180;
imu_data.Yaw = imu_data.Yaw - imu_data.Yaw(1) + yaw0;

coordinates = read_TS_data(fullfile(data_dir, TSfile), idx1, idx2);
coordinates(:, 1) = coordinates(:, 1) - coordinates(1, 1) + t0_TS;
coordinates(:, 4) = coordinates(:, 4) + TS_h;
coordinates = leverarm_corr_TS(coordinates, leverarm, imu_data);

% Convert Xsens output to the NED attitude convention used by Lambert4.
imu_data.Pitch = -imu_data.Pitch;
imu_data.Yaw = 90 - imu_data.Yaw;

[~, n] = size(fid3);
if n == 1
    fid3 = fid3';
end

[t, fft_result] = VLP_preprocess_highfreq(fid3, nLED, fhz, fs, dt, rate);
num_point = length(t);
X = zeros(num_point, nLED + 3);

% Data aligned for RSS prediction from total-station truth and IMU attitude.
[common_t, interp_roll, interp_pitch, interp_yaw, interpolated_coords] = ...
    alignIMUandCoordinates(t, imu_data, coordinates);
p = [NED2ENU(interpolated_coords), interp_roll, interp_pitch, interp_yaw];

options = optimoptions('lsqnonlin', 'Display', 'off');
lb = [0, 0, 0];
ub = [10, 8, 3];
X0 = [x0, y0, h];

for k = 1:num_point
    [~, idx] = min(abs(imu_data.Time - t(k)));
    roll = imu_data.Roll(idx) / 180 * pi;
    pitch = imu_data.Pitch(idx) / 180 * pi;
    yaw = imu_data.Yaw(idx) / 180 * pi;

    objFun = @(x) Lambert4(x, LED, a, M, roll, pitch, yaw, fft_result(k, :), std);
    [sol, ~, residual] = lsqnonlin(objFun, X0, lb, ub, options);

    X(k, 1:3) = sol;
    X(k, 4:3+nLED) = residual';
end
fprintf('解算完成。\n');

nav_enu = X(:, 1:3);
ref_enu = coordinates(:, 2:4);
led_ned = enu2ned(LED);
nav_ned = enu2ned(nav_enu);
ref_ned = enu2ned(ref_enu);

[t_err, err_enu] = calc_position_error(t(:), nav_enu, coordinates(:, 1), ref_enu);
err_ned = enu2ned(err_enu);
err_3d = vecnorm(err_ned, 2, 2);

stats = calc_error_stats(err_ned, err_3d);
print_error_stats(stats);

RSS_pred = preRSSrpy(p, a, M, NED2ENU(LED));
RSS_obs_common = interp1(t(:), fft_result, common_t(:), 'linear');
RSS_residual = RSS_obs_common - RSS_pred;

set_plot_defaults();
fig_dir = fullfile(repo_root, 'output', 'figures', 'vlpins_loose');
ensure_dir(fig_dir);

plot_trajectory_ned(nav_ned, ref_ned, led_ned, fig_dir);
plot_position_and_error(t(:), nav_ned, coordinates(:, 1), ref_ned, ...
    t_err, err_ned, err_3d, stats, fig_dir);
plot_rss_diagnostics(t(:), fft_result, common_t(:), RSS_pred, RSS_residual, nLED, fig_dir);
plot_imu_diagnostics(imu_data, fig_dir);

output(VLPfile, TSfile, t, X, fft_result, coordinates);

function ned = enu2ned(enu)
    ned = zeros(size(enu));
    if isequal(size(enu), [3, 1])
        ned(1) = enu(2);
        ned(2) = enu(1);
        ned(3) = -enu(3);
    else
        ned(:, 1) = enu(:, 2);
        ned(:, 2) = enu(:, 1);
        ned(:, 3) = -enu(:, 3);
    end
end

function [t_common, err_enu, nav_interp_enu, ref_common_enu] = calc_position_error(t_nav, nav_enu, t_ref, ref_enu)
    t_nav = t_nav(:);
    t_ref = t_ref(:);
    in_range = (t_ref >= min(t_nav)) & (t_ref <= max(t_nav));
    t_common = t_ref(in_range);
    ref_common_enu = ref_enu(in_range, :);

    if isempty(t_common)
        error('定位结果与全站仪真值没有共同时间范围。');
    end

    nav_interp_enu = interp1(t_nav, nav_enu, t_common, 'linear');
    err_enu = nav_interp_enu - ref_common_enu;
end

function stats = calc_error_stats(err_ned, err_3d)
    stats.mean_3d = mean(err_3d);
    stats.rmse_3d = sqrt(mean(err_3d.^2));
    stats.max_3d = max(err_3d);
    stats.mean_ned = mean(err_ned, 1);
    stats.rmse_ned = sqrt(mean(err_ned.^2, 1));
    stats.max_abs_ned = max(abs(err_ned), [], 1);
end

function print_error_stats(stats)
    fprintf('mean error 3d: %.6f m\n', stats.mean_3d);
    fprintf('rmse error 3d: %.6f m\n', stats.rmse_3d);
    fprintf('max error 3d : %.6f m\n', stats.max_3d);
    fprintf('mean ned error: N=%.6f m, E=%.6f m, D=%.6f m\n', ...
        stats.mean_ned(1), stats.mean_ned(2), stats.mean_ned(3));
    fprintf('rmse ned error: N=%.6f m, E=%.6f m, D=%.6f m\n', ...
        stats.rmse_ned(1), stats.rmse_ned(2), stats.rmse_ned(3));
    fprintf('max abs ned  : N=%.6f m, E=%.6f m, D=%.6f m\n', ...
        stats.max_abs_ned(1), stats.max_abs_ned(2), stats.max_abs_ned(3));
end

function set_plot_defaults()
    set(groot, 'defaultAxesFontName', 'Microsoft YaHei');
    set(groot, 'defaultTextFontName', 'Microsoft YaHei');
    set(groot, 'defaultLegendFontName', 'Microsoft YaHei');
    set(groot, 'defaultAxesFontSize', 16);
    set(groot, 'defaultTextFontSize', 18);
    set(groot, 'defaultLegendFontSize', 14);
    set(groot, 'defaultLineLineWidth', 1.8);
    set(groot, 'defaultAxesLineWidth', 1.2);
end

function style_axes(ax)
    grid(ax, 'on');
    box(ax, 'on');
    ax.LineWidth = 1.2;
    ax.FontSize = 16;
    ax.TitleFontSizeMultiplier = 1.1;
    ax.LabelFontSizeMultiplier = 1.0;
end

function ensure_dir(out_dir)
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
end

function fig = make_paper_figure(name, size_cm)
    fig = figure('Name', name, 'Color', 'w', 'Units', 'centimeters', ...
        'Position', [2, 2, size_cm(1), size_cm(2)], ...
        'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, size_cm(1), size_cm(2)], ...
        'PaperSize', size_cm);
end

function export_paper_figure(fig, out_dir, file_name)
    png_file = fullfile(out_dir, [file_name, '.png']);
    pdf_file = fullfile(out_dir, [file_name, '.pdf']);

    try
        exportgraphics(fig, png_file, 'Resolution', 600, 'BackgroundColor', 'white');
        exportgraphics(fig, pdf_file, 'ContentType', 'vector', 'BackgroundColor', 'white');
    catch
        print(fig, png_file, '-dpng', '-r600');
        try
            print(fig, pdf_file, '-dpdf', '-vector');
        catch
            warning('PDF export failed for %s.', file_name);
        end
    end
end

function lgd = paper_legend(varargin)
    lgd = legend(varargin{:});
    lgd.FontSize = 14;
    lgd.Box = 'on';
end

function plot_trajectory_ned(nav_ned, ref_ned, led_ned, fig_dir)
    fig = make_paper_figure('轨迹对比', [24, 18]);
    plot3(nav_ned(:, 1), nav_ned(:, 2), nav_ned(:, 3), '.-', 'DisplayName', 'VLP/INS轨迹');
    hold on;
    plot3(ref_ned(:, 1), ref_ned(:, 2), ref_ned(:, 3), '-', 'DisplayName', '全站仪真值');
    plot3(nav_ned(1, 1), nav_ned(1, 2), nav_ned(1, 3), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', '起点');
    plot3(nav_ned(end, 1), nav_ned(end, 2), nav_ned(end, 3), 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', '终点');
    scatter3(led_ned(:, 1), led_ned(:, 2), led_ned(:, 3), 60, 'r', '*', 'DisplayName', 'LED');
    xlabel('N (m)');
    ylabel('E (m)');
    zlabel('D (m)');
    title('NED坐标系轨迹对比');
    paper_legend('Location', 'best');
    axis equal;
    view(3);
    style_axes(gca);
    export_paper_figure(fig, fig_dir, 'trajectory_ned');
end

function plot_position_and_error(t_nav, nav_ned, t_ref, ref_ned, ...
    t_err, err_ned, err_3d, stats, fig_dir)
    labels = {'N', 'E', 'D'};

    fig_pos = make_paper_figure('NED位置时序', [24, 18]);
    tl = tiledlayout(fig_pos, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'NED位置时序');
    ax_pos = gobjects(3, 1);
    for i = 1:3
        ax_pos(i) = nexttile;
        plot(t_nav, nav_ned(:, i), 'DisplayName', '估计值');
        hold on;
        plot(t_ref, ref_ned(:, i), 'DisplayName', '真值');
        ylabel(sprintf('%s (m)', labels{i}));
        title(sprintf('%s方向位置', labels{i}));
        if i == 1
            paper_legend('Location', 'best', 'NumColumns', 2);
        end
        style_axes(ax_pos(i));
    end
    xlabel(ax_pos(end), '时间 (s)');
    linkaxes(ax_pos, 'x');
    export_paper_figure(fig_pos, fig_dir, 'position_ned');

    fig_err = make_paper_figure('NED与3D误差时序', [24, 20]);
    tl = tiledlayout(fig_err, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'NED与3D误差时序');
    ax_err = gobjects(4, 1);
    for i = 1:3
        ax_err(i) = nexttile;
        plot(t_err, err_ned(:, i), 'DisplayName', sprintf('%s误差', labels{i}));
        hold on;
        yline(0, 'k:', 'DisplayName', '零误差');
        ylabel('误差 (m)');
        title(sprintf('%s方向误差', labels{i}));
        if i == 1
            paper_legend('Location', 'best', 'NumColumns', 2);
        end
        style_axes(ax_err(i));
    end

    ax_err(4) = nexttile;
    plot(t_err, err_3d, 'DisplayName', '3D误差');
    hold on;
    yline(stats.mean_3d, '--', 'DisplayName', sprintf('平均值 %.3f m', stats.mean_3d));
    yline(stats.rmse_3d, '-.', 'DisplayName', sprintf('RMSE %.3f m', stats.rmse_3d));
    xlabel('时间 (s)');
    ylabel('误差 (m)');
    title(sprintf('3D误差：平均 %.3f m，RMSE %.3f m，最大 %.3f m', ...
        stats.mean_3d, stats.rmse_3d, stats.max_3d));
    paper_legend('Location', 'best');
    style_axes(ax_err(4));
    linkaxes(ax_err, 'x');
    export_paper_figure(fig_err, fig_dir, 'error_ned_3d');
end

function plot_rss_diagnostics(t, rss_obs, common_t, rss_pred, rss_residual, nLED, fig_dir)
    rss_height = max(14, 4.2 * nLED);
    fig_rss = make_paper_figure('RSS观测与预测对比', [24, rss_height]);
    tl = tiledlayout(fig_rss, nLED, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'RSS观测与预测对比');
    ax_rss = gobjects(nLED, 1);

    for i = 1:nLED
        ax_rss(i) = nexttile;
        plot(t, rss_obs(:, i), 'DisplayName', '观测RSS');
        hold on;
        plot(common_t, rss_pred(:, i), 'DisplayName', '预测RSS');
        ylabel(sprintf('LED%d RSS', i));
        title(sprintf('LED%d RSS对比', i));
        if i == 1
            paper_legend('Location', 'best', 'NumColumns', 2);
        end
        style_axes(ax_rss(i));
    end
    xlabel(ax_rss(end), '时间 (s)');
    linkaxes(ax_rss, 'x');
    export_paper_figure(fig_rss, fig_dir, 'rss_observed_predicted');

    fig_res = make_paper_figure('RSS残差', [24, rss_height]);
    tl = tiledlayout(fig_res, nLED, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'RSS残差');
    ax_res = gobjects(nLED, 1);

    for i = 1:nLED
        ax_res(i) = nexttile;
        plot(common_t, rss_residual(:, i), 'DisplayName', '观测-预测');
        hold on;
        yline(0, 'k:', 'DisplayName', '零残差');
        ylabel('RSS残差');
        title(sprintf('LED%d RSS残差', i));
        if i == 1
            paper_legend('Location', 'best', 'NumColumns', 2);
        end
        style_axes(ax_res(i));
    end

    xlabel(ax_res(end), '时间 (s)');
    linkaxes(ax_res, 'x');
    export_paper_figure(fig_res, fig_dir, 'rss_residual');
end

function plot_imu_diagnostics(imu_data, fig_dir)
    t_imu = imu_data.Time;
    acc = [imu_data.Acc_X, imu_data.Acc_Y, imu_data.Acc_Z];
    gyr = [imu_data.Gyr_X, imu_data.Gyr_Y, imu_data.Gyr_Z];
    euler = [imu_data.Roll, imu_data.Pitch, imu_data.Yaw];
    acc_norm = vecnorm(acc, 2, 2);
    gyr_norm = vecnorm(gyr, 2, 2);

    fig = make_paper_figure('IMU诊断', [24, 20]);
    tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'IMU数据诊断');
    ax = gobjects(6, 1);

    ax(1) = nexttile;
    plot(t_imu, acc);
    title('加速度三轴');
    ylabel('加速度 (m/s^2)');
    paper_legend('Acc X', 'Acc Y', 'Acc Z', 'Location', 'best');
    style_axes(ax(1));

    ax(2) = nexttile;
    plot(t_imu, gyr);
    title('角速度三轴');
    ylabel('角速度 (rad/s)');
    paper_legend('Gyr X', 'Gyr Y', 'Gyr Z', 'Location', 'best');
    style_axes(ax(2));

    ax(3) = nexttile;
    plot(t_imu, euler);
    title('欧拉角');
    ylabel('角度 (deg)');
    paper_legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    style_axes(ax(3));

    ax(4) = nexttile;
    plot(t_imu, acc_norm);
    title('加速度模值');
    ylabel('|Acc| (m/s^2)');
    style_axes(ax(4));

    ax(5) = nexttile;
    plot(t_imu, gyr_norm);
    title('角速度模值');
    xlabel('时间 (s)');
    ylabel('|Gyr| (rad/s)');
    style_axes(ax(5));

    ax(6) = nexttile;
    plot(t_imu, imu_data.PacketCounter);
    title('数据包计数器');
    xlabel('时间 (s)');
    ylabel('PacketCounter');
    style_axes(ax(6));

    linkaxes(ax, 'x');
    export_paper_figure(fig, fig_dir, 'imu_diagnostics');
end
