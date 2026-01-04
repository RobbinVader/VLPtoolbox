# 可见光定位(VLP)工具箱

这是一个用于可见光定位 (VLP) 及其相关多传感器融合（IMU、全站仪真值）的数据处理、数据分析与算法工具箱。包含实时数据采集脚本以及用于离线处理、算法验证、坐标转换和精度评估的 MATLAB 函数库。

@author: Sun Xiao, Wu Haoyu, Wuhan University  

@测试版本：MATLAB 2024a/2024b，Windows 11

## 功能列表

### postprocess 目录

MATLAB后处理程序

- **sync.m**: 用于VLP、IMU、全站仪真值数据时间同步

### function 目录
用于MATLAB数据处理的函数库

- **read_newest_VLP.m**: 自动查找指定目录下修改时间最新的 VLP 数据文件，便于批量处理时快速定位最新数据
- **VLP_t0.m**: 从标准格式的文件名（如 YYYYMMDD_HHMMSS...）中提取初始时间戳，用于多传感器时间同步
- **read_imu_data.m**: 读取由 Xsens MT Manager 导出的 IMU 数据文件; 解析加速度、角速度、欧拉角、时间戳等信息并存入结构体
- **read_TS_data.m**: 读取全站仪（Total Station）导出的坐标真值数据; 根据点号筛选数据，并提取时间与坐标 (X, Y, Z)
- **read_TS_MS60GS.m**: 专用于读取 MS60GS 全站仪导出的 .FRT 格式数据; 使用正则表达式解析包含时间戳的高精度轨迹真值
- **VLP_preprocess_highfreq.m**: 对原始 VLP 信号进行 FFT 处理以提取 RSS，不加汉明窗
- **VLP_preprocess_highfreq_ham.m**: 对原始 VLP 信号加汉明窗后进行 FFT 处理，以提取更平滑的 RSS 数据
- **RSS_corr.m**: 基于载体运动状态（速度、角速度）和几何关系对 RSS 进行动态修正，用于消除动态过程中的信号偏差
- **preRSS.m**: 根据给定的位置、光照模型参数（a, M）和 LED 坐标，理论预测 RSS 值
- **Lambert2.m**: 朗伯函数，用于水平标定
- **Lambert3.m**: 朗伯函数，用于3D定位
- **Lambert4.m**: 朗伯函数，用于包含姿态的3D定位
- **alignIMUandCoordinates.m**: 对齐 IMU 数据和全站仪真值数据的时间轴, 通过插值将两者统一到共同的时间基准上，便于误差分析
- **leverarm_corr_TS.m**: 利用 IMU 的偏航角信息，修正全站仪棱镜中心与 IMU 中心之间的物理偏移（杆臂），获取真实的设备中心轨迹
- **NED2ENU.m**: NED和ENU系之间转换
- **Euler2Dcm.m**: NED系下欧拉角到方向余弦矩阵转换
- **Euler2Dcm_ENU.m**: ENU系下欧拉角到方向余弦矩阵转换
- **accuracy_eval.m**: 对比解算轨迹与参考真值，并绘制CDF曲线
- **plot3Dtraj.m**: 绘制3D轨迹、LED位置和真值轨迹
- **plot_export_imu_data.m**: 绘制IMU数据，并输出文件
- **VLP_parameter.m**: 读取VLP参数配置
- **config_from_yaml**: 读取数据配置文件

### realtime 目录
python脚本，用于实时显示及处理VLP数据

- **real_time_main.py**：实时接收数据并进行三维定位主程序，通过串口读取光强数据，绘制RSS变化与散点平面轨迹等图表
- **real_time_show_rss.py**: 最初版本的实时脚本，作为纪念保留
- **utils.py**：工具函数模块，封装了三维定位、RSS计算、读取配置文件参数、接受串口数据、保存数据至文件等功能函数

### test_haoyu 目录
一些@Haoyu Wu进行功能测试的脚本，与工具箱使用无关

### thirdparty 第三方库

- **MartinKoch123-yaml-1.6.0.0**: 用于配置文件yaml的解析，无需安装，可直接调用

### config 目录

- **VLP_parameter.yaml**: 用于可见光定位的基本参数

### data 目录
存放实验采集数据，以实验日期为区分

## 安装说明

要运行 real_time_show_rss.py 脚本，需要安装以下 Python 依赖包：

```bash
pip install pyserial numpy scipy matplotlib pyyaml
```

               🌟
              🎄🎄
             🎄🔴🎄
            🎄✨🎄🎄
           🎄🎄🟡🎄🎄
          🎄🎄🎄🎄🎄🎄
         🎄🔵🎄🎄🎄🎄🎄
        🎄🎄🎄🎄🎄🔴🎄🎄
       🎄✨🎄🎄🎄🎄🎄✨🎄
      🎄🎄🟡🎄🎄🎄🔵🎄🎄🎄
     🎄🎄🎄🎄🎄🎄🎄🎄🎄🎄🎄
               🪵🪵
               🪵🪵
    ----------------------------
    🎊 HAPPY NEW YEAR 2026 !! 🎊