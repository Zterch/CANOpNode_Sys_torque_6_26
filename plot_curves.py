#!/usr/bin/env python3
"""
曲线绘制脚本 - 绘制电机转速、压力传感器、电流实际值和电流命令曲线
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def plot_curves(csv_file):
    """读取CSV文件并绘制曲线"""
    
    # 读取CSV文件
    df = pd.read_csv(csv_file)
    
    # 去除列名中的空格
    df.columns = df.columns.str.strip()
    
    # 去除时间列中的空格并转换
    df['Time'] = df['Time'].str.strip()
    df['Time'] = pd.to_datetime(df['Time'], format='%H:%M:%S.%f')
    start_time = df['Time'].iloc[0]
    df['Time_sec'] = (df['Time'] - start_time).dt.total_seconds()
    
    # 创建图形 - 使用2x1子图布局
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # 子图1: 电机实际转速
    ax1 = axes[0]
    ax1.plot(df['Time_sec'], df['MotorSpeed(rpm)'], 'b-', linewidth=1.5, label='Motor Speed (rpm)')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Motor Speed (rpm)', fontsize=12, color='b')
    ax1.tick_params(axis='y', labelcolor='b')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=10)
    ax1.set_title('Motor Speed', fontsize=14, fontweight='bold')
    
    # 子图2: 压力传感器、电流实际值和电流命令
    ax2 = axes[1]
    
    # 绘制压力传感器数据（左Y轴）
    line1 = ax2.plot(df['Time_sec'], df['Pressure(kg)'], 'g-', linewidth=1.5, label='Pressure (kg)')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Pressure (kg)', fontsize=12, color='g')
    ax2.tick_params(axis='y', labelcolor='g')
    ax2.grid(True, alpha=0.3)
    
    # 创建右Y轴用于电流数据
    ax2_right = ax2.twinx()
    line2 = ax2_right.plot(df['Time_sec'], df['Current(A)'], 'r-', linewidth=1.5, label='Current Actual (A)')
    line3 = ax2_right.plot(df['Time_sec'], df['TargetCurrent(A)'], 'm--', linewidth=1.5, label='Current Command (A)')
    ax2_right.set_ylabel('Current (A)', fontsize=12, color='r')
    ax2_right.tick_params(axis='y', labelcolor='r')
    
    # 合并图例
    lines = line1 + line2 + line3
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc='upper left', fontsize=10)
    ax2.set_title('Pressure Sensor & Current Data', fontsize=14, fontweight='bold')
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图片到当前目录
    import os
    csv_basename = os.path.basename(csv_file)
    output_file = csv_basename.replace('.csv', '_curves.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Curves saved to: {output_file}")
    
    # 显示图形
    plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_curves.py <csv_file>")
        print("Example: python3 plot_curves.py logdata/gravity_data_20260521_163330.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    plot_curves(csv_file)

if __name__ == '__main__':
    main()
