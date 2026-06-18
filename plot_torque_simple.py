#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
力矩响应曲线绘制脚本 - 简化版
只生成图片，不显示图表

使用方法:
  python3 plot_torque_simple.py <csv文件路径>
  
如果不指定文件，会自动查找最新的CSV文件
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from glob import glob

# 设置字体
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

def find_latest_csv(log_dir="/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys_torque_6_15/logdata"):
    """查找最新的CSV文件"""
    csv_files = glob(os.path.join(log_dir, "gravity_data_*.csv"))
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getmtime)

def main():
    # 获取CSV文件路径
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        print("Finding latest CSV file...")
        csv_file = find_latest_csv()
        if csv_file is None:
            print("Error: No CSV file found")
            sys.exit(1)
        print(f"Found: {csv_file}")
    
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    # 加载数据
    print(f"Loading data...")
    df = pd.read_csv(csv_file)
    df['Time_sec'] = df.index * 0.01  # 100Hz
    
    # 检查列
    if 'TargetTorque(Nm)' not in df.columns or 'ActualTorque(Nm)' not in df.columns:
        print("Error: Missing torque columns in CSV")
        sys.exit(1)
    
    # 计算指标
    mask = df['TargetTorque(Nm)'] != 0
    if mask.sum() > 0:
        target = df.loc[mask, 'TargetTorque(Nm)']
        actual = df.loc[mask, 'ActualTorque(Nm)']
        error = target - actual
        
        print("\n" + "="*50)
        print("Torque Control Performance Metrics")
        print("="*50)
        print(f"Max Error: {np.max(np.abs(error)):.4f} Nm")
        print(f"Mean Error: {np.mean(error):.4f} Nm")
        print(f"RMS Error: {np.sqrt(np.mean(error**2)):.4f} Nm")
        print(f"Correlation: {np.corrcoef(target, actual)[0, 1]:.4f}")
        print("="*50)
    
    # 绘图
    print("\nGenerating plot...")
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    time = df['Time_sec']
    target = df['TargetTorque(Nm)']
    actual = df['ActualTorque(Nm)']
    error = target - actual
    
    # 图1: 目标 vs 实际力矩
    ax1 = axes[0]
    ax1.plot(time, target, 'b-', label='Target Torque', linewidth=1.5)
    ax1.plot(time, actual, 'r-', label='Actual Torque', linewidth=1.5)
    ax1.set_ylabel('Torque (Nm)')
    ax1.set_title('Target vs Actual Torque')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 图2: 误差
    ax2 = axes[1]
    ax2.plot(time, error, 'g-', label='Tracking Error', linewidth=1.2)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax2.fill_between(time, error, alpha=0.3, color='green')
    ax2.set_ylabel('Error (Nm)')
    ax2.set_title('Torque Tracking Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 图3: 电机速度
    ax3 = axes[2]
    if 'MotorSpeed(rpm)' in df.columns:
        ax3.plot(time, df['MotorSpeed(rpm)'], 'm-', label='Motor Speed', linewidth=1.2)
        ax3.set_ylabel('Speed (rpm)')
        ax3.set_title('Motor Speed Response')
        ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_xlabel('Time (sec)')
    
    plt.tight_layout()
    
    # 保存
    base_name = os.path.splitext(csv_file)[0]
    save_path = base_name + "_torque_response.png"
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"\nPlot saved: {save_path}")
    print("Done!")

if __name__ == "__main__":
    main()
