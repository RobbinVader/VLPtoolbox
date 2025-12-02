function [VLPfile, TSfile, IMUfile, idx1, idx2, t0_TS, t0_IMU, std] = config_from_yaml(cfgfile)
% 从 YAML 配置文件加载实验参数
    addpath("../thirdparty/MartinKoch123-yaml-1.6.0.0");
    % 加载 YAML 文件
    cfg = yaml.loadFile(cfgfile);
    
    VLPfile    = cfg.VLPfile;
    TSfile     = cfg.TSfile;
    IMUfile    = cfg.IMUfile;
    idx1       = cfg.idx1;
    idx2       = cfg.idx2;
    t0_TS      = cfg.t0_TS;
    t0_IMU     = cfg.t0_IMU;
    std        = cfg.std;
end