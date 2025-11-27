>> 这是一个用于可见光定位 (VLP) 及其相关多传感器融合（IMU、全站仪真值）的数据处理与算法工具箱。包含实时数据采集脚本以及用于离线处理、算法验证、坐标转换和精度评估的 MATLAB 函数库
>> @author: The Lord SunXiao, Wuhan University
>> @date: 11/27/2025

-function:
--read_newest_VLP.m: 自动查找指定目录下修改时间最新的 VLP 数据文件，便于批量处理时快速定位最新数据
--VLP_t0.m: 从标准格式的文件名（如 YYYYMMDD_HHMMSS...）中提取初始时间戳，用于多传感器时间同步
--read_imu_data.m: 读取由 Xsens MT Manager 导出的 IMU 数据文件; 解析加速度、角速度、欧拉角、时间戳等信息并存入结构体
--read_TS_data.m: 读取全站仪（Total Station）导出的坐标真值数据; 根据点号筛选数据，并提取时间与坐标 (X, Y, Z)
--read_TS_MS60GS.m: 专用于读取 MS60GS 全站仪导出的 .FRT 格式数据; 使用正则表达式解析包含时间戳的高精度轨迹真值
--VLP_preprocess_highfreq.m: 对原始 VLP 信号进行 FFT 处理以提取 RSS，不加汉明窗
--VLP_preprocess_highfreq_ham.m: 对原始 VLP 信号加汉明窗后进行 FFT 处理，以提取更平滑的 RSS 数据
--RSS_corr.m: 基于载体运动状态（速度、角速度）和几何关系对 RSS 进行动态修正，用于消除动态过程中的信号偏差
--preRSS.m: 根据给定的位置、光照模型参数（a, M）和 LED 坐标，理论预测 RSS 值
--Lambert2.m: 用于水平标定
--Lambert3.m: 用于3D定位
--Lambert4.m: 用于包含姿态的3D定位
--alignIMUandCoordinates.m: 对齐 IMU 数据和全站仪真值数据的时间轴, 通过插值将两者统一到共同的时间基准上，便于误差分析
--leverarm_corr_TS.m: 利用 IMU 的偏航角信息，修正全站仪棱镜中心与 IMU 中心之间的物理偏移（杆臂），获取真实的设备中心轨迹
--NED2ENU.m
--Euler2Dcm.m
--Euler2Dcm_ENU.m
--accuracy_eval.m: 对比解算轨迹与参考真值
--plot3Dtraj.m
--plot_export_imu_data.m

-realtime:
--real_time_show_rss.py: 通过串口实时读取光强传感器数据，并绘制RSS变化等图表
