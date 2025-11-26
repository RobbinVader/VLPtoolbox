function [t_common,err]=accuracy_eval(t,nav,t_ref,ref)
% accuracy_eval - 评估导航数据相对于参考真值的精度并绘图
%
% 输入：
%   t      : [N x 1] 导航时间戳（单调递增）
%   nav    : [N x 3] 导航位置数据（对应 x, y, z）
%   t_ref  : [M x 1] 参考真值时间戳（单调递增）
%   ref    : [M x 3] 参考真值位置数据（对应 x, y, z）
%
% 输出：
%   t_common   : 共同时间戳（即 t_ref 中落在 t 范围内的部分）
%   err        : 误差向量 = nav_interp - ref_common [K x 3]

    % 输入检查
    assert(isvector(t) && iscolumn(t), 't must be a column vector');
    assert(size(nav,1) == length(t) && size(nav,2) == 3, ...
           'nav must be N x 3 where N = length(t)');
    assert(isvector(t_ref) && iscolumn(t_ref), 't_ref must be a column vector');
    assert(size(ref,1) == length(t_ref) && size(ref,2) == 3, ...
           'ref must be M x 3 where M = length(t_ref)');

    % 确定共同时间范围：取 t_ref 中在 [min(t), max(t)] 内的时间戳
    t_min = min(t);
    t_max = max(t);
    in_range = (t_ref >= t_min) & (t_ref <= t_max);
    
    t_common = t_ref(in_range);
    ref_common = ref(in_range, :);

    % 如果没有共同时间戳，返回空
    if isempty(t_common)
        nav_interp = [];
        err = [];
        return;
    end

    % 对 nav 的每一列进行插值（线性插值）
    nav_interp = zeros(length(t_common), 3);
    for i = 1:3
        nav_interp(:, i) = interp1(t, nav(:, i), t_common, 'linear');
    end

    % 计算误差向量
    err = nav_interp - ref_common;
    
    figure;
    subplot(2,1,1);plot(t_common,err);
    legend('X','Y','Z');
    grid on;
    set(gca,'FontSize',10,'LineWidth',1,'Box','on');
    set(get(gca, 'xlabel'), 'Color', [0,0,0])
    set(get(gca, 'ylabel'), 'Color', [0,0,0])
    set(gca, 'XColor', [0,0,0])
    set(gca, 'YColor', [0,0,0])
    xlabel('Time (s)','FontSize',12);
    ylabel('Error','FontSize',12);

    % 绘制CDF
    dM = sort(vecnorm(err, 2, 2));
    subplot(2,1,2);
    h1 = cdfplot(dM);
    set(h1,'Color',[0.47,0.67,0.14],'LineWidth',1)
    ts = sprintf('CDF (%d points)',length(dM));
    title(ts);
    set(gca,'FontSize',10,'LineWidth',1,'Box','on');
    set(get(gca, 'xlabel'), 'Color', [0,0,0])
    set(get(gca, 'ylabel'), 'Color', [0,0,0])
    set(gca, 'XColor', [0,0,0])
    set(gca, 'YColor', [0,0,0])
    xlabel('Horizontal error (m)','FontSize',12);
    ylabel('Fraction of distribution','FontSize',12);
    
    % CDF的分位点
    dM50 = median(dM);
    hold on;plot(dM50,.5,'.r','handlevisibility','off');
    ts = sprintf(' 50%% %.2fm',dM50);
    text(dM50,.5,ts);
    
    dM67 = dM(round(length(dM)*.67));
    hold on;plot(dM67,.67,'.r','handlevisibility','off');
    ts = sprintf(' 67%% %.2fm',dM67);
    text(dM67,.67,ts,'VerticalAlignment','Top');
    
    dM95 = dM(round(length(dM)*.95));
    hold on;plot(dM95,.95,'.r','handlevisibility','off');
    ts = sprintf(' 95%% %.2fm',dM95);
    text(dM95,.95,ts,'VerticalAlignment','Top');
    
    dM100 = max(dM);
    hold on;plot(dM100,1,'.r','handlevisibility','off');
    ts = sprintf(' 100%% %.2fm',dM100);
    text(dM100,1,ts,'VerticalAlignment','Top');