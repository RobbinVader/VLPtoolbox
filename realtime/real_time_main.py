# -*- coding: UTF-8 -*-
import os.path
import threading
import queue
import time
import serial
import numpy as np
from scipy.fftpack import fft
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.optimize import least_squares
from utils import *

# ==========================================
# 1. 原始 Lambert3 函数 (保持不变)
# ==========================================
# def Lambert3(x, LED, a, M, Rss, std):
#     residuals = []
#     for i in range(len(a)):
#         los = LED[i]-x
#         s = np.linalg.norm(los)
#         if s == 0:
#             cos_alpha = 0
#         else:
#             cos_alpha = los[2]/s
        
#         if cos_alpha > 0:
#             p_rec = a[i] * (cos_alpha ** (M[i] + 1)) / (s ** 2)
#         else:
#             p_rec = 0
        
#         res = (p_rec - Rss[i]) / std[i]
#         residuals.append(res)
#     return np.array(residuals)

# def read_serial_data(ser, data_queue):
#     st = time.time()
#     while True:
#         if ser.in_waiting > 0:
#             if time.time() - st > 1000000:
#                 return
#             data = ser.read(2001) 
#             if len(data) != 0:
#                 data_queue.put(data)

# def save_data_to_file(data_queue, filename, light_queue):
#     with open(filename, mode='ab') as file:
#         while True:
#             data = data_queue.get()
#             light_queue.put(data)
#             file.write(data)
#             file.flush()
#             data_queue.task_done()

# def save_RSS_to_file(data_queue, filename):
#     with open(filename, mode='ab') as file:
#         while True:
#             RSS = data_queue.get()
#             if len(RSS) != 0:
#                 np.savetxt(file, RSS, fmt='%f', delimiter=',')
#                 file.flush()
#             data_queue.task_done()

if __name__ == '__main__':
    try:
        # --- 串口配置 ---
        portx = "COM9"
        bps = 115200
        timex = 10
        ser = serial.Serial(portx, bps, timeout=timex)

        # --- 文件保存设置 ---
        dt = time.time()
        dtime = time.localtime(dt)
        h, m, s = str(dtime[3]).zfill(2), str(dtime[4]).zfill(2), str(dtime[5]).zfill(2)
        filename = f"{dtime[0]}{dtime[1]}{dtime[2]}_{h}_{m}_{s}.txt"
        
        record_dir = "./"
        file_sign = "static"
        if not os.path.isdir(record_dir):
            os.mkdir(record_dir)

        record_file_light = record_dir + "_Light_" + file_sign + "_" + filename
        record_file_rss = record_dir + "_RSS_" + file_sign + "_" + filename

        # --- VLP 参数设置 ---
        Send_freq = [735, 215, 640, 305, 865, 520]
        Nled = len(Send_freq)
        Receive_freq = 2000
        sampleSize = 2001
        sampleTime = 10000000
        FFT_dt = 1
        win_size = FFT_dt * Receive_freq
        RSS = np.zeros((1, Nled))
        plotWindow_size = 60  # seconds
        
        # --- 标定参数 ---
        LED = np.array([
            [4.5604, 0.7996, 2.99],
            [4.2862, 2.1105, 2.99],
            [4.5802, 3.4361, 2.99],
            [6.5215, 3.1602, 2.99],
            [6.5806, 2.1225, 2.99],
            [6.6521, 0.9161, 2.99]
        ])
        a = np.array([152.1205, 138.0382, 167.5161, 138.3205, 121.9980, 133.0103])
        M = np.array([0.4313, 0.3485, 0.5762, 0.7756, 0.5558, 0.9801])
        std = np.array([1, 1, 1, 1, 1, 1])
        X0 = np.array([4.5604, 0.7996, 1.0])
        X = X0

        # ==========================================
        # 2. 初始化单窗口布局 (1x2 布局)
        # ==========================================
        plt.ion()
        fig = plt.figure(figsize=(16, 8))
        plt.rcParams['font.sans-serif'] = ['SimHei'] # 设置字体为黑体，解决中文乱码
        plt.rcParams['axes.unicode_minus'] = False # 解决负号 '-' 显示为方块的问题
        fig.canvas.manager.set_window_title('VLP 实时监控 (最近60秒)')
        
        # 使用 GridSpec 将窗口分割: 左列(轨迹) vs 右列(RSS等图表)
        # 3行 x 2列
        gs = gridspec.GridSpec(3, 2, width_ratios=[1, 1])

        # --- 左侧: 轨迹图 (占用第0列的所有3行) ---
        ax_traj = fig.add_subplot(gs[:, 0])
        ax_traj.set_title("2D 轨迹 (最近60秒)")
        ax_traj.set_xlabel('X (m)')
        ax_traj.set_ylabel('Y (m)')
        ax_traj.set_xlim(3, 8)  
        ax_traj.set_ylim(0, 5)
        ax_traj.grid(True)
        # 绘制静态元素 (LED位置)
        ax_traj.plot(LED[:, 0], LED[:, 1], 'kx', markersize=10, label='LEDs')
        # 初始化动态元素 (路径和当前点)
        line_traj, = ax_traj.plot([], [], 'r.-', label='Path')
        point_curr, = ax_traj.plot([], [], 'bo', markersize=8, label='Current')
        ax_traj.legend(loc='upper right')

        # --- 右侧顶部: 频谱图 (当前帧) ---
        ax_fft = fig.add_subplot(gs[0, 1])
        ax_fft.set_title("频谱 (当前帧)")
        ax_fft.grid(True)

        # --- 右侧中部: RSS 趋势 ---
        ax_rss = fig.add_subplot(gs[1, 1])
        ax_rss.set_title("RSS 趋势 (最近60秒)")
        ax_rss.grid(True)

        # --- 右侧底部: 均值变化 ---
        ax_evn = fig.add_subplot(gs[2, 1])
        ax_evn.set_title("均值变化 (最近60秒)")
        ax_evn.grid(True)
        
        plt.tight_layout()

        # 数据缓存变量
        traj_x = []
        traj_y = []
        time_stamps = []   # 用于记录时间戳，实现滑动窗口
        evn = []
        total_RSS = np.empty((0, Nled)) # 初始化空二维数组
        
        data_queue = queue.Queue()
        light_queue = queue.Queue()
        rss_queue = queue.Queue()

        # 启动线程
        save_thread_1 = threading.Thread(target=save_data_to_file, args=(data_queue, record_file_light, light_queue))
        save_thread_1.daemon = True
        save_thread_1.start()

        save_thread_2 = threading.Thread(target=save_RSS_to_file, args=(rss_queue, record_file_rss))
        save_thread_2.daemon = True
        save_thread_2.start()

        Total_data = ''
        LightData = []
        sign_start = 0 
        lk = 1
        ds = time.time() # 记录开始时间
        
        print("Start loop...")

        while True:
            # --- 数据读取与解析 ---
            while len(LightData) < Receive_freq:
                data = ser.read(sampleSize)
                if len(data) != 0:
                    data_queue.put(data)
                
                try:
                    s = data.decode('utf-8')
                except:
                    s = ""
                Total_data += s
                temp_data = Total_data.split(",")
                LightData = temp_data

            if sign_start == 0:
                LightData = LightData[1:]
                sign_start = 1
            lk += 1
            
            # 提取窗口数据
            temp = LightData[0:win_size]
            print("Processing frame, length:", len(temp))
            
            # 剩余数据处理
            temp2 = LightData[win_size:]
            Total_data = ','.join(temp2)
            LightData = []
            
            DataSize = len(temp)
            TimporalData = np.zeros((DataSize, 1))
            for i in range(DataSize):
                try:
                    val = temp[i]
                    TimporalData[i] = int(val) if val != '' else 0
                except:
                    TimporalData[i] = 0
            
            # --- 核心处理逻辑 ---
            current_time_rel = time.time() - ds # 计算相对时间
            
            # 1. 计算均值
            current_mean = np.mean(TimporalData)
            evn.append(current_mean)
            
            # 2. FFT 处理
            w = np.hamming(DataSize)
            TimporalData_flat = TimporalData.flatten()
            sf = w * TimporalData_flat
            yy = fft(sf.T, DataSize)
            yreal = np.abs(yy)
            mag = yreal * 2 / DataSize * 1.852
            
            # 3. 提取 RSS
            for j in range(Nled):
                idx = int((Send_freq[j] * DataSize / Receive_freq))
                start_idx_fft = max(0, idx - 5)
                end_idx_fft = min(len(mag), idx + 5)
                RSS[0, j] = max(mag[start_idx_fft : end_idx_fft])
            
            rss_queue.put(RSS)
            
            # 存储 RSS 历史数据
            total_RSS = np.vstack((total_RSS, RSS))

            # 4. 定位解算
            lb, ub = [0, 0, 0], [10, 8, 3]
            result = least_squares(
                Lambert3,
                X,
                bounds=(lb, ub),
                args=(LED, a, M, RSS[0], std),
                verbose=0
            )
            
            if result.success:
                X = result.x
                traj_x.append(X[0])
                traj_y.append(X[1])
                print(f"Time: {current_time_rel:.1f}s | Pos: {X[0]:.2f}, {X[1]:.2f}, {X[2]:.2f}")
            else:
                # 如果解算失败，依然追加数据以保持长度一致（这里追加上一次的位置或当前位置）
                traj_x.append(X[0])
                traj_y.append(X[1])

            time_stamps.append(current_time_rel)

            # ==========================================
            # 3. 绘图 (最近60秒逻辑)
            # ==========================================
            
            # 设定时间阈值：当前时间 - 60秒
            threshold_time = current_time_rel - plotWindow_size
            
            # 查找时间戳中第一个大于阈值的索引 (start_idx)
            start_idx = 0
            for i, t in enumerate(time_stamps):
                if t > threshold_time:
                    start_idx = i
                    break
            
            # 对所有数据列表进行切片，只保留最近60秒
            plot_t = time_stamps[start_idx:]
            plot_traj_x = traj_x[start_idx:]
            plot_traj_y = traj_y[start_idx:]
            plot_rss = total_RSS[start_idx:, :]
            plot_evn = evn[start_idx:]

            # --- 更新轨迹图 ---
            line_traj.set_data(plot_traj_x, plot_traj_y)
            point_curr.set_data([X[0]], [X[1]])
            
            # --- 更新频谱图 (不需要历史数据，仅显示当前帧) ---
            ax_fft.cla()
            ax_fft.set_title("频谱 (当前帧)")
            mag_plot = mag.copy()
            mag_plot[0:6] = 0 # 过滤直流分量
            mag_plot[-6:] = 0
            ax_fft.stem(mag_plot, linefmt='b-', markerfmt='b.', basefmt='r-')
            ax_fft.grid(True)

            # --- 更新 RSS 趋势图 ---
            ax_rss.cla()
            ax_rss.set_title("RSS 趋势 (最近60秒)")
            colors = ["red", "yellow", "green", "blue", "purple", "gray"]
            # 绘制各 LED 的 RSS 变化
            for k in range(Nled):
                ax_rss.plot(plot_t, plot_rss[:, k], color=colors[k], label=f"LED{k+1}")
            ax_rss.legend(loc='upper right', fontsize='small', ncol=3)
            ax_rss.grid(True)
            # 动态调整 x 轴范围
            ax_rss.set_xlim(left=max(0, threshold_time), right=current_time_rel + 1)

            # --- 更新均值趋势图 ---
            ax_evn.cla()
            ax_evn.set_title("均值变化 (最近60秒)")
            ax_evn.plot(plot_t, plot_evn, 'k-')
            ax_evn.grid(True)
            ax_evn.set_xlim(left=max(0, threshold_time), right=current_time_rel + 1)

            plt.pause(0.01)

    except Exception as e:
        print("---异常---：", e)
        import traceback
        traceback.print_exc()
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial Closed.")