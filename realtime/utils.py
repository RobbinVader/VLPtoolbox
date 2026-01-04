import time
import serial
import numpy as np
import yaml

def Lambert3(x, LED, a, M, Rss, std):
    residuals = []
    for i in range(len(a)):
        los = LED[i]-x
        s = np.linalg.norm(los)
        if s == 0:
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
    st = time.time()
    while True:
        if ser.in_waiting > 0:
            if time.time() - st > 1000000:
                return
            data = ser.read(2001) 
            if len(data) != 0:
                data_queue.put(data)

def save_data_to_file(data_queue, filename, light_queue):
    with open(filename, mode='ab') as file:
        while True:
            data = data_queue.get()
            light_queue.put(data)
            file.write(data)
            file.flush()
            data_queue.task_done()

def save_RSS_to_file(data_queue, filename):
    with open(filename, mode='ab') as file:
        while True:
            RSS = data_queue.get()
            if len(RSS) != 0:
                np.savetxt(file, RSS, fmt='%f', delimiter=',')
                file.flush()
            data_queue.task_done()

def load_vlp_config(yaml_path):
    """
    从YAML文件中读取VLP系统参数
    """
    with open(yaml_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # 1. 读取基础参数
    Send_freq = config['fhz']
    Nled = config['nLED']
    
    # 2. 读取FFT相关参数
    Receive_freq = config['fft']['fs']
    FFT_dt = config['fft']['window_duration']
    
    # 3. 读取标定参数 (转换为numpy array)
    LED = np.array(config['LED'])
    # 注意：这里指定读取 'dynamic' 模式下的 a 和 M
    a = np.array(config['calibration']['dynamic']['a'])
    M = np.array(config['calibration']['dynamic']['M'])

    return {
        "Send_freq": Send_freq,
        "Nled": Nled,
        "Receive_freq": Receive_freq,
        "FFT_dt": FFT_dt,
        "LED": LED,
        "a": a,
        "M": M
    }

def calculate_rss(Nled: int, Send_freq, DataSize, Receive_freq, mag, RSS):
    for i in range(Nled):
        idx = int((Send_freq[i] * DataSize / Receive_freq))
        start_idx_fft = max(0, idx - 5)
        end_idx_fft = min(len(mag), idx + 5)
        RSS[0, i] = max(mag[start_idx_fft : end_idx_fft])  # 这里面我直接对RSS做的全局修改，严谨点的话可以用局部变量