function [LED,nLED,a,M,fhz,fs,dt,rate]=VLP_parameter(cfgfile,m)

addpath("../thirdparty/MartinKoch123-yaml-1.6.0.0");
cfg=yaml.loadFile(cfgfile);

% 提取参数
nLED = cfg.nLED;

LED = zeros(nLED, 3);
for i = 1:nLED
    LED(i, :) = [cfg.LED{i}{:}];  % 展开 inner cell
end

if m == 1
    a = cell2mat(cfg.calibration.planar.a);
    M = cell2mat(cfg.calibration.planar.M);
elseif m == 2
    a = cell2mat(cfg.calibration.height_diff.a);
    M = cell2mat(cfg.calibration.height_diff.M);
elseif m == 3
    a = cell2mat(cfg.calibration.dynamic.a);
    M = cell2mat(cfg.calibration.dynamic.M);
end

fhz = cell2mat(cfg.fhz);
fs = cfg.fft.fs;
dt = cfg.fft.window_duration;
rate = cfg.fft.rss_sampling_rate;