import matplotlib.pyplot as plt
import numpy as np
import os
import matplotlib as mpl

# 设置全局样式
plt.style.use('default')  # 使用简单的白色背景
mpl.rcParams['font.family'] = 'DejaVu Sans'
mpl.rcParams['axes.unicode_minus'] = False
mpl.rcParams['axes.grid'] = True
mpl.rcParams['grid.linestyle'] = '--'
mpl.rcParams['grid.alpha'] = 0.5
mpl.rcParams['axes.linewidth'] = 0.5

# 设置全局刻度标签大小
mpl.rcParams['xtick.labelsize'] = 16  # X轴刻度标签字号
mpl.rcParams['ytick.labelsize'] = 16  # Y轴刻度标签字号

# 文件路径
file_path = "/home/mark/iros/code/xin_ok/error_ours2.txt"

def read_error_data(file_path):
    """读取误差数据文件"""
    errors = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                try:
                    error = float(line.strip())
                    errors.append(error)
                except ValueError:
                    continue
    except Exception as e:
        print(f"Error reading file: {e}")
        exit(1)
    return np.array(errors)

def plot_errors(errors):
    """绘制简洁的误差曲线图（类似您提供的图片）"""
    if len(errors) == 0:
        print("No data to plot")
        return
    
    # 创建图形
    fig, ax = plt.subplots(figsize=(8, 5), dpi=300)
    
    # 创建时间序列索引 (每500行对应1秒)
    time_seconds = np.arange(len(errors)) / 390.0
    
    # 绘制曲线 - 单一蓝色线条
    ax.plot(time_seconds, errors, 'b-', linewidth=1.2, alpha=0.9)
    
    # 设置标题和标签 (英文)
    ax.set_xlabel('Time (s)', fontsize=18)  # 已调整到18号字
    #ax.set_ylabel('e', fontsize=18)  # 取消注释并添加Y轴标签
    
    # 设置坐标轴范围
    ax.set_xlim(0, time_seconds[-1])
    ax.set_ylim(0, max(errors) * 1.1)
    
    # 设置x轴刻度为每1.0秒一个标记
    max_time = time_seconds[-1]
    xticks = np.arange(0, max_time + 1.0, 1.0)
    ax.set_xticks(xticks)
    
    # 设置y轴刻度标签字号（通过直接设置刻度属性）
    ax.tick_params(axis='both', which='major', labelsize=18)  # 增加刻度标签字号
    
    # 添加网格
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # 优化布局
    plt.tight_layout()
    
    # 显示图形
    plt.show()

if __name__ == "__main__":
    # 读取数据
    errors = read_error_data(file_path)
    
    # 绘制简洁的曲线图
    plot_errors(errors)
