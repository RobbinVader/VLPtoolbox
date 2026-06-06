function [best_t0_IMU, best_delay, metrics] = auto_sync_imu_vlp(cfgfile)
% 自动估计 IMU 与 VLP 的时间差，并输出推荐的 t0_IMU
% 仅打印结果，不会修改 YAML 配置文件

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(script_dir);
addpath(fullfile(repo_root, 'function'));
addpath(fullfile(repo_root, 'thirdparty', 'MartinKoch123-yaml-1.6.0.0'));

if nargin < 1
    cfgfile = fullfile(repo_root, 'config', '20251127_6.yaml');
end

[VLPfile, TSfile, IMUfile, idx1, idx2, t0_TS, t0_IMU, ~] = ...
    config_from_yaml(cfgfile);
[LED, nLED, a, M, fhz, fs, dt, rate] = ...
    VLP_parameter(fullfile(repo_root, 'config', 'VLP_parameter.yaml'), 3);

TS_h = 1.5963;
leverarm = [0.11; 0.10; -0.02];

[t_vlp, rss_obs, t0_VLP] = build_vlp_rss_series(fullfile(repo_root, 'data', VLPfile), VLPfile, nLED, fhz, fs, dt, rate);

coordinates = read_TS_data(fullfile(repo_root, 'data', TSfile), idx1, idx2);
coordinates(:, 1) = coordinates(:, 1) - coordinates(1, 1) + t0_TS;
coordinates(:, 4) = coordinates(:, 4) + TS_h;

imu_data = read_imu_data(fullfile(repo_root, 'data', IMUfile));
imu_data.Yaw = imu_data.Yaw + 180;

coarse = estimate_imu_ts_offset(imu_data, coordinates, t0_IMU);

imu_data.Time = imu_data.Time + coarse.best_t0_IMU;
coordinates_corr = leverarm_corr_TS(coordinates, leverarm, imu_data);

[common_t, interp_roll, interp_pitch, interp_yaw, interpolated_coords] = ...
    alignIMUandCoordinates(t_vlp, imu_data, coordinates_corr);
state = [NED2ENU(interpolated_coords), interp_roll, interp_pitch, interp_yaw];
rss_pred = preRSSrpy(state, a, M, NED2ENU(LED));

fine = search_vlp_delay_by_rss(t_vlp, rss_obs, common_t, rss_pred);

best_t0_IMU = coarse.best_t0_IMU + fine.best_delay;
best_delay = best_t0_IMU - t0_IMU;

metrics = struct();
metrics.coarse_delay = coarse.best_t0_IMU - t0_IMU;
metrics.fine_delay = fine.best_delay;
metrics.best_score = fine.best_score;
metrics.coarse_score = coarse.best_score;
metrics.search_range = fine.search_range;
metrics.overlap_duration = fine.overlap_duration;
metrics.is_reliable = fine.is_reliable;

fprintf('==== 自动时间同步结果 ====\n');
fprintf('VLP 初始时间: %.3f s\n', t0_VLP);
fprintf('原始 t0_IMU: %.3f s\n', t0_IMU);
fprintf('粗对齐后 t0_IMU: %.3f s\n', coarse.best_t0_IMU);
fprintf('推荐 t0_IMU: %.3f s\n', best_t0_IMU);
fprintf('相对原配置修正量: %.3f s\n', best_delay);
fprintf('精匹配得分: %.4f\n', fine.best_score);
fprintf('有效重叠时长: %.2f s\n', fine.overlap_duration);
fprintf('结果可靠性: %d\n', fine.is_reliable);

plot_auto_sync_diagnostics(t_vlp, rss_obs, common_t, rss_pred, coarse, fine, t0_IMU, best_t0_IMU, nLED);

end

function plot_auto_sync_diagnostics(t_vlp, rss_obs, common_t, rss_pred, coarse, fine, old_t0_IMU, best_t0_IMU, nLED)
figure('Name', 'IMU-VLP 自动时间同步诊断', 'Position', [100, 100, 1200, 900]);
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(coarse.t_coord, coarse.ts_feature, 'LineWidth', 1.2);
hold on;
plot(coarse.t_coord, coarse.imu_feature_before, '--', 'LineWidth', 1.0);
plot(coarse.t_coord, coarse.imu_feature_after, 'LineWidth', 1.0);
legend('TS特征', 'IMU特征-原始配置', 'IMU特征-粗对齐后', 'Location', 'best');
title(sprintf('粗对齐: t0\\_IMU %.3f -> %.3f s', old_t0_IMU, coarse.best_t0_IMU));
ylabel('Normalized feature');
grid on;

nexttile;
plot(fine.candidates, fine.scores, 'LineWidth', 1.2);
hold on;
xline(fine.best_delay, '--r', 'LineWidth', 1.2);
title(sprintf('精搜索分数曲线, best delay = %.3f s', fine.best_delay));
xlabel('Delay relative to coarse t0\_IMU (s)');
ylabel('Score');
grid on;

nexttile;
max_led_plot = min(nLED, 3);
colors = lines(max_led_plot);
legend_entries = cell(1, 3 * max_led_plot);
for i = 1:max_led_plot
    pred_before = interp1(common_t, rss_pred(:, i), t_vlp, 'linear', NaN);
    pred_after = interp1(common_t, rss_pred(:, i), t_vlp - fine.best_delay, 'linear', NaN);
    obs_norm = normalize_signal(rss_obs(:, i));
    plot(t_vlp, obs_norm, 'Color', colors(i, :), 'LineWidth', 1.0);
    hold on;
    plot(t_vlp, normalize_signal(pred_before), '--', 'Color', colors(i, :), 'LineWidth', 0.8);
    plot(t_vlp, normalize_signal(pred_after), ':', 'Color', colors(i, :), 'LineWidth', 1.2);
    legend_entries{3 * i - 2} = sprintf('Obs LED%d', i);
    legend_entries{3 * i - 1} = sprintf('Pred before LED%d', i);
    legend_entries{3 * i} = sprintf('Pred after LED%d', i);
end
title(sprintf('RSS 对齐前后对比, 推荐 t0\\_IMU = %.3f s', best_t0_IMU));
xlabel('Time (s)');
ylabel('Normalized RSS');
legend(legend_entries, 'Location', 'eastoutside');
grid on;
end

function x = normalize_signal(x)
x = x(:);
mu = mean(x, 'omitnan');
sigma = std(x, 'omitnan');
if sigma < 1e-8
    x = zeros(size(x));
else
    x = (x - mu) / sigma;
end
end

function [t, rss_obs, t0_VLP] = build_vlp_rss_series(filepath, VLPfile, nLED, fhz, fs, dt, rate)
vlp = load(filepath);
vlp = vlp(2:end-1);
if size(vlp, 2) == 1
    vlp = vlp';
end

t0_VLP = VLP_t0(VLPfile, 0);
[t, rss_obs] = VLP_preprocess_highfreq_ham(vlp, nLED, fhz, fs, dt, rate);
end

function result = estimate_imu_ts_offset(imu_data, coordinates, initial_t0_IMU)
t_coord = coordinates(:, 1);
xyz = coordinates(:, 2:4);

dt_coord = gradient(t_coord);
dt_coord(dt_coord == 0) = median(dt_coord(dt_coord > 0));
velocity = [gradient(xyz(:, 1))./dt_coord, gradient(xyz(:, 2))./dt_coord, gradient(xyz(:, 3))./dt_coord];
ts_feature = normalize_signal(sqrt(sum(velocity.^2, 2)));

t_imu_raw = imu_data.Time(:);
imu_feature_raw = normalize_signal(sqrt(imu_data.Gyr_X(:).^2 + imu_data.Gyr_Y(:).^2 + imu_data.Gyr_Z(:).^2));

search_half_width = 20;
coarse_step = 0.05;
candidates = (initial_t0_IMU-search_half_width):coarse_step:(initial_t0_IMU+search_half_width);
scores = -inf(size(candidates));

for k = 1:numel(candidates)
    imu_at_ts = interp1(t_imu_raw + candidates(k), imu_feature_raw, t_coord, 'linear', NaN);
    scores(k) = feature_score(ts_feature, imu_at_ts);
end

[best_score, best_idx] = max(scores);
best_t0_IMU = candidates(best_idx);

result = struct();
result.best_t0_IMU = best_t0_IMU;
result.best_score = best_score;
result.candidates = candidates;
result.scores = scores;
result.t_coord = t_coord;
result.ts_feature = ts_feature;
result.imu_feature_before = interp1(t_imu_raw + initial_t0_IMU, imu_feature_raw, t_coord, 'linear', NaN);
result.imu_feature_after = interp1(t_imu_raw + best_t0_IMU, imu_feature_raw, t_coord, 'linear', NaN);
end

function result = search_vlp_delay_by_rss(t_vlp, rss_obs, common_t, rss_pred)
search_half_width = 5;
fine_step = 0.02;
candidates = -search_half_width:fine_step:search_half_width;
scores = -inf(size(candidates));
overlap_durations = zeros(size(candidates));

for k = 1:numel(candidates)
    [scores(k), overlap_durations(k)] = rss_delay_score(t_vlp, rss_obs, common_t, rss_pred, candidates(k));
end

[best_score, best_idx] = max(scores);
best_delay = candidates(best_idx);

result = struct();
result.best_delay = best_delay;
result.best_score = best_score;
result.candidates = candidates;
result.scores = scores;
result.search_range = [candidates(1), candidates(end)];
result.overlap_duration = overlap_durations(best_idx);
result.is_reliable = isfinite(best_score) && result.overlap_duration > 2 && best_score > 0.2;
end

function [score, overlap_duration] = rss_delay_score(t_vlp, rss_obs, common_t, rss_pred, delay)
nLED = size(rss_obs, 2);
led_scores = NaN(1, nLED);
valid_any = false(size(t_vlp(:)));

for i = 1:nLED
    pred = interp1(common_t, rss_pred(:, i), t_vlp(:) - delay, 'linear', NaN);
    obs = rss_obs(:, i);
    valid = isfinite(obs) & isfinite(pred);
    valid_any = valid_any | valid;
    if nnz(valid) >= 10
        led_scores(i) = feature_score(obs(valid), pred(valid));
    end
end

score = mean(led_scores, 'omitnan');
if isnan(score)
    score = -inf;
end

if any(valid_any)
    t_overlap = t_vlp(valid_any);
    overlap_duration = max(t_overlap) - min(t_overlap);
else
    overlap_duration = 0;
end
end

function score = feature_score(a, b)
a = normalize_signal(a);
b = normalize_signal(b);
valid = isfinite(a) & isfinite(b);
if nnz(valid) < 5
    score = -inf;
    return;
end

a = a(valid);
b = b(valid);
den = norm(a) * norm(b);
if den < 1e-8
    score = -inf;
else
    score = (a' * b) / den;
end
end
