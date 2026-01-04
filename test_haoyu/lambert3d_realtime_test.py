# -*- coding: UTF-8 -*-
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from realtime.utils import *

# ==========================================
# 1. 原始 Lambert3 函数 (完全保留，未修改)
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

# ==========================================
# 2. 标定参数 (从原始代码复制)
# ==========================================
LED=[
    [4.5604, 0.7996, 2.99],
    [4.2862, 2.1105, 2.99],
    [4.5802, 3.4361, 2.99],
    [6.5215, 3.1602, 2.99],
    [6.5806, 2.1225, 2.99],
    [6.6521, 0.9161, 2.99]
]
LED = np.array(LED)
a = np.array([152.1205, 138.0382, 167.5161, 138.3205, 121.9980, 133.0103])
M = np.array([0.4313,   0.3485,   0.5762,   0.7756,   0.5558,   0.9801])
std = np.array([1,1,1,1,1,1]) 
X0 = np.array([4.5604, 0.7996, 1.0]) # 初始位置

# ==========================================
# 3. 主测试逻辑
# ==========================================
def test_reconstruction(filename):
    print(f"正在读取文件: {filename} ...")
    
    try:
        with open(filename, 'r') as f:
            content = f.read()
            # 预处理：将换行符替换为逗号，以防数据跨多行
            content = content.replace('\n', ',')
            # 分割并转换为浮点数，过滤掉空字符
            data_list = [float(x) for x in content.split(',') if x.strip()]
    except FileNotFoundError:
        print(f"错误: 找不到文件 '{filename}'。请确保文件存在且路径正确。")
        return
    except ValueError:
        print("错误: 文件包含无法转换为数字的字符。请检查数据格式。")
        return

    data_array = np.array(data_list)
    total_len = len(data_array)
    
    # 按照每6个数据一个历元进行重塑
    if total_len % 6 != 0:
        print(f"警告: 数据总长度 {total_len} 不是 6 的倍数，末尾 {total_len % 6} 个数据将被丢弃。")
        valid_len = (total_len // 6) * 6
        data_array = data_array[:valid_len]
    
    rss_epochs = data_array.reshape(-1, 6)
    num_epochs = len(rss_epochs)
    print(f"成功加载数据，共 {num_epochs} 个历元 (Epochs)。开始计算...")

    # 轨迹存储
    traj_x = []
    traj_y = []
    
    # 初始位置猜测 (Warm start)
    current_pos = X0.copy()
    
    # 边界约束 (x, y, z)，防止 z < 0
    lb = [-np.inf, -np.inf, 0]
    ub = [np.inf, np.inf, np.inf]

    for i in range(num_epochs):
        rss_obs = rss_epochs[i]
        
        # 调用最小二乘法
        result = least_squares(
            Lambert3,
            current_pos,
            # bounds=(lb, ub),
            args=(LED, a, M, rss_obs, std),
            verbose=0
        )
        
        if result.success:
            current_pos = result.x
            traj_x.append(current_pos[0])
            traj_y.append(current_pos[1])
        else:
            # 如果计算失败，可以选择沿用上一次的位置或者记录错误
            pass

    # ==========================================
    # 4. 绘制结果
    # ==========================================
    plt.figure(figsize=(8, 6))
    
    # 绘制 LED 灯的位置作为参考
    plt.plot(LED[:,0], LED[:,1], 'kx', markersize=10, label='LEDs')
    
    # 绘制重建轨迹
    plt.plot(traj_x, traj_y, 'r.-', linewidth=1, markersize=2, label='Reconstructed Path')
    
    # 设置图表格式
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Lambert3 Function Reconstruction Test')
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # 保持XY轴比例一致
    
    plt.show()

if __name__ == '__main__':
    # 请在这里修改你的输入文件名
    DATA_FILENAME = "data/20251204/_RSS_static_2025124_14_17_20.txt"
    
    # 如果你想测试，可以先创建一个假的 rss_data.txt，否则请确保目录下有该文件
    test_reconstruction(DATA_FILENAME)