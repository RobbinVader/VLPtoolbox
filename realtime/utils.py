import time
import serial
import numpy as np

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