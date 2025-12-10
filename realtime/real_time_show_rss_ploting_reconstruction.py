# -*- coding: UTF-8 -*-
import os.path
import threading

import serial  # 导入模块
import numpy as np
from scipy.fftpack import fft,ifft
import matplotlib.pyplot as plt
import time
import queue

# ==========================================
# 1. 你的原始 Lambert3 函数 (完全保留)
# ==========================================
def Lambert3(x, LED, a, M, Rss, std):
    residuals = []
    for i in range(len(a)):
        los = LED[i]-x
        s = np.linalg.norm(los)
        if s== 0:
            cos_alpha = 0
        else:
            cos_alpha = los[2]/s
        
        if cos_alpha > 0:
            p_rec = a[i] * (cos_alpha ** (M[i] + 1)) / (s ** 2)
        else:
            p_rec = 0
        
        res = (p_rec - Rss[i]) / std[i]
        residuals.append(res)
    return np.array(residuals)

def read_serial_data(ser, data_queue):
    # 这里sampleSize未定义，为了保持代码完整性，这里暂不修改逻辑
    # 实际运行时需确保sampleSize可访问
    st = time.time()
    while True:
        if ser.in_waiting > 0:
            if time.time() - st > 1000000:
                return
            data = ser.read(2001) # 假设 sampleSize=2001
            print("read data length = ", len(data))
            if len(data) !=0:
                data_queue.put(data)

def save_data_to_file(data_queue, filename,light_queue):
    with open(filename, mode='ab') as file:
        while True:
            data = data_queue.get()
            light_queue.put(data)
            file.write(data)
            print("save data length = ", len(data))
            file.flush()  # 立即刷新缓冲区，确保数据写入文件
            data_queue.task_done()

def save_RSS_to_file(data_queue, filename):
    with open(filename, mode='ab') as file:
        while True:
            RSS = data_queue.get()
            if len(RSS)!=0:
                np.savetxt(file, RSS, fmt='%f', delimiter=',')
                file.flush()
            data_queue.task_done()


if __name__ == '__main__':
    try:
        from scipy.optimize import least_squares # 导入最小二乘

        portx = "COM9" # 端口
        # bps = 921600 #921600 # 波特率
        # bps = 1000000  # 921600 # 波特率
        bps = 115200
        timex = 10 # 超时设置

        # 定义和打开串口
        ser = serial.Serial(portx, bps, timeout=timex)

        #### 获取当前时间用于定义文件名 ####
        dt = time.time()
        dtime = time.localtime(dt)
        h = str(dtime[3])
        m = str(dtime[4])
        s = str(dtime[5])
        if len(h) == 1: h = '0' + h
        if len(m) == 1: m = '0' + m
        if len(s) == 1: s = '0' + s

        # 定义文件
        filename = str(dtime[0]) + str(dtime[1]) + str(dtime[2]) + '_' + h + '_' + m + '_' + s + '.txt';

        # 打开文件
        record_dir = "./" # 文件目录
        file_sign = "static"   # 自定义标记
        if os.path.isdir(record_dir):
            print("record_dir exist")
        else:
            os.mkdir(record_dir)

        record_file_light = record_dir + "_Light_" +file_sign+ "_" +filename
        record_file_rss = record_dir + "_RSS_" +file_sign+ "_" +filename

        ### 定义VLP参数 ###
        Send_freq = [735,215,640,305,865,520] # LED灯频率
        Nled = len(Send_freq)
        Receive_freq = 2000 # 采样率
        sampleSize = 2001  # 串口接收数据频率
        sampleTime = 10000000  # 采样时常
        FFT_dt = 1
        win_size = FFT_dt*Receive_freq
        RSS = np.zeros((1, Nled))
        
        # 标定参数
        LED=[
            [4.5604, 0.7996, 2.99],
            [4.2862, 2.1105, 2.99],
            [4.5802, 3.4361, 2.99],
            [6.5215, 3.1602, 2.99],
            [6.5806, 2.1225, 2.99],
            [6.6521, 0.9161, 2.99]
        ]
        LED = np.array(LED)
        a=np.array([152.1205, 138.0382, 167.5161, 138.3205, 121.9980, 133.0103])
        M=np.array([0.4313,   0.3485,   0.5762,   0.7756,   0.5558,   0.9801])
        std = np.array([1,1,1,1,1,1]) # 后续可以改进
        X0=np.array([4.5604, 0.7996, 1.0]) # 初始位置
        X = X0

        # ==========================================
        # 2. 初始化两个独立窗口
        # ==========================================
        plt.ion()  # 打开交互模式
        
        # --- 窗口1: 原来的信号/RSS显示 ---
        fig1 = plt.figure(num=1, figsize=(6, 8))
        fig1.canvas.manager.set_window_title('Signal Monitor')
        # 不需要手动添加subplot，下面用plt.subplot直接画
        
        # --- 窗口2: 新增的轨迹显示 ---
        fig2 = plt.figure(num=2, figsize=(6, 6))
        fig2.canvas.manager.set_window_title('2D Trajectory')
        ax_traj = fig2.add_subplot(111)
        
        # 初始化轨迹图背景
        ax_traj.set_xlabel('X (m)')
        ax_traj.set_ylabel('Y (m)')
        ax_traj.set_xlim(0, 8) # 根据场地大小调整
        ax_traj.set_ylim(0, 5) # 根据场地大小调整
        ax_traj.grid(True)
        # 画灯的位置做参考
        ax_traj.plot(LED[:,0], LED[:,1], 'kx', markersize=10, label='LEDs')
        
        # 初始化轨迹线条对象 (比每次清空重画要快)
        line_traj, = ax_traj.plot([], [], 'r.-', label='Path') 
        point_curr, = ax_traj.plot([], [], 'bo', markersize=8, label='Current')
        ax_traj.legend()
        
        # 轨迹数据缓存
        traj_x = []
        traj_y = []

        # 存储历史RSS值
        total_RSS = np.array([])
        evn = []
        sign = 0 #标记第一次存RSS值
        ds = time.time()


        # 定义和打开串口
        data_queue = queue.Queue()  # 创建一个队列用于数据传递
        light_queue = queue.Queue()  # 创建一个队列用于数据传递
        rss_queue = queue.Queue()  # 创建一个队列用于数据传递

        # 创建并启动数据保存线程
        save_thread = threading.Thread(target=save_data_to_file, args=(data_queue, record_file_light,light_queue))
        save_thread.daemon = True
        save_thread.start()

        # 存RSS线程
        save_thread = threading.Thread(target=save_RSS_to_file, args=(rss_queue, record_file_rss))
        save_thread.daemon = True
        save_thread.start()

        # 画图线程变量
        Total_data = ''
        LightData = []
        sign_start = 0 
        lk = 1
        
        print("Start loop...")

        while (True):
            # if time.time() - ds > sampleTime:
            #     break

            while(len(LightData)<Receive_freq):
                ## 串口读取数据
                data = ser.read(sampleSize)
                print(data)
                if lk == 2:
                    print(1)
                ## 存储数据
                if len(data) != 0:
                    data_queue.put(data)

                ## 将采集所获数据转为numpy 数组
                try:
                    s = data.decode('utf-8')
                except:
                    s = "" # 防止解码错误
                Total_data = Total_data + s
                # 分割字符
                temp_data = Total_data.split(",")
                LightData = temp_data

            if sign_start == 0:
                # 丢掉最后一个空字符
                LightData = LightData[1:]
                sign_start = 1
            lk = lk +1
            temp = LightData[0:win_size]
            print("length of temp = ", len(temp))

            temp2 = LightData[win_size:]
            Total_data = ','.join(temp2)
            LightData = []
            # 获取字符长度
            DataSize = len(temp)
            N = DataSize

            # char 转为 int 类型
            TimporalData = np.zeros((DataSize, 1))
            for i in range(DataSize):
                try:
                    if temp[i] == '':
                        TimporalData[i] = 0
                    else:
                        TimporalData[i] = int(temp[i])
                except:
                    TimporalData[i] = 0
            evn.append(np.mean(TimporalData))

            #### FFT #####
            w = np.hamming(DataSize)
            TimporalData_flat = TimporalData.flatten() # 展平以避免FFT警告
            sf = w * TimporalData_flat

            # FFT
            yy = fft(sf.T, DataSize)
            yreal = np.abs(yy)
            mag = yreal * 2 / N * 1.852
            n = np.arange(0, DataSize, 1)
            f1 = n * Receive_freq / N

            # 提取相对应的RSS值
            for j in range(Nled):
                idx = int((Send_freq[j] * N / Receive_freq))
                # 简单越界保护
                start_idx = max(0, idx - 5)
                end_idx = min(len(mag), idx + 5)
                RSS[0, j] = max(mag[start_idx : end_idx])
            
            print("RSS is ", RSS)
            rss_queue.put(RSS)

            # 三维定位置只需要用一行RSS
            # 添加了 Z > 0 的物理约束，防止跑到地下
            lb = [0, 0, 0]
            ub = [10, 8, 3]
            
            result = least_squares(
                Lambert3,
                X,
                bounds=(lb, ub),
                args=(LED, a, M, RSS[0], std),  # 你的修正：RSS[0] 正确
                verbose=0
            )
            
            computingStatus = result.success
            if computingStatus:
                X = result.x
                # 记录轨迹点
                traj_x.append(X[0])
                traj_y.append(X[1])
                print(f"Pos: {X[0]:.2f}, {X[1]:.2f}, {X[2]:.2f}")

            # 如果第一次则
            if sign == 0:
                total_RSS = RSS
                sign = 1
            else:
                total_RSS = np.vstack((total_RSS, RSS))
            
            # ==========================================
            # 3. 更新窗口 1 (原来的信号图)
            # ==========================================
            plt.figure(1) # 切换到窗口1上下文
            
            plt.subplot(3, 1, 1)
            plt.cla()
            mag_plot = mag.copy()
            mag_plot[0:6] = 0
            mag_plot[-6:] = 0
            plt.stem(mag_plot, linefmt='b-', markerfmt='b.', basefmt='r-')

            plt.subplot(3, 1, 2)
            plt.cla()
            colors = ["red", "yellow", "green", "blue", "purple", "gray"]
            # 限制绘图长度，防止卡顿
            plot_len = min(100, total_RSS.shape[0])
            for k in range(len(Send_freq)):
                plt.plot(total_RSS[-plot_len:, k], color=colors[k], label=str(Send_freq[k]))

            plt.rcParams['font.sans-serif'] = ['SimHei'] 
            plt.rcParams['axes.unicode_minus'] = False 
            plt.grid(True)
            plt.legend(["LED1",  "LED2", "LED3", "LED4", "LED5", "LED6"], ncol=6, fontsize='small')
            
            plt.subplot(3, 1, 3)
            plt.cla()
            plt.plot(np.array(evn)) # 绘制均值变化

            # ==========================================
            # 4. 更新窗口 2 (轨迹图)
            # ==========================================
            plt.figure(2) # 切换到窗口2上下文
            
            # 使用 set_data 更新数据，非常快
            line_traj.set_data(traj_x, traj_y)
            point_curr.set_data([X[0]], [X[1]]) # 注意必须是列表
            
            plt.pause(0.1) # 刷新所有窗口
            
            dt1 = time.time()
            print("---------------",(dt1-ds))

    except Exception as e:
        print("---异常---：", e)
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial Closed.")