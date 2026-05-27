#!/usr/bin/env python3
"""
压力传感器滤波效果对比脚本
绘制原始压力和滤波后压力的对比曲线
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
from datetime import datetime

def plot_pressure_comparison(csv_file):
    """绘制原始压力和滤波后压力的对比图"""
    
    # 读取CSV数据
    df = pd.read_csv(csv_file, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    
    # 创建时间序列（秒）- 从0开始
    df['Time_sec'] = np.arange(len(df)) * 0.02  # 50Hz采样
    
    # 提取压力数据
    pressure_raw = df['PressureRaw(kg)']
    pressure_filtered = df['PressureFiltered(kg)']
    f0 = df['F0(kg)'].iloc[0]  # 基准压力
    
    # 计算差值（滤波效果）
    filter_diff = pressure_raw - pressure_filtered
    
    print("=" * 80)
    print("压力传感器滤波效果分析")
    print("=" * 80)
    print(f"\n数据文件: {csv_file}")
    print(f"数据点数: {len(df)}")
    print(f"时间跨度: {df['Time_sec'].iloc[-1]:.2f} 秒")
    print(f"采样频率: 50 Hz")
    print(f"\n基准压力 (F0): {f0:.3f} kg")
    
    print(f"\n【原始压力统计】")
    print(f"  最小值: {pressure_raw.min():.3f} kg")
    print(f"  最大值: {pressure_raw.max():.3f} kg")
    print(f"  平均值: {pressure_raw.mean():.3f} kg")
    print(f"  标准差: {pressure_raw.std():.3f} kg")
    print(f"  峰峰值: {pressure_raw.max() - pressure_raw.min():.3f} kg")
    
    print(f"\n【滤波后压力统计】")
    print(f"  最小值: {pressure_filtered.min():.3f} kg")
    print(f"  最大值: {pressure_filtered.max():.3f} kg")
    print(f"  平均值: {pressure_filtered.mean():.3f} kg")
    print(f"  标准差: {pressure_filtered.std():.3f} kg")
    print(f"  峰峰值: {pressure_filtered.max() - pressure_filtered.min():.3f} kg")
    
    print(f"\n【滤波效果】")
    print(f"  标准差降低: {(1 - pressure_filtered.std()/pressure_raw.std())*100:.1f}%")
    print(f"  峰峰值降低: {(1 - (pressure_filtered.max()-pressure_filtered.min())/(pressure_raw.max()-pressure_raw.min()))*100:.1f}%")
    print(f"  最大差值: {filter_diff.abs().max():.3f} kg")
    print(f"  平均差值: {filter_diff.abs().mean():.3f} kg")
    
    # 创建图表
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    fig.suptitle('Pressure Filter Comparison - Raw vs Filtered (Notch Filter)', fontsize=16, fontweight='bold')
    
    # 图1: 原始压力和滤波后压力对比
    ax1 = axes[0]
    ax1.plot(df['Time_sec'], pressure_raw, label='Raw Pressure', color='red', 
             linewidth=1.5, alpha=0.7, linestyle='--')
    ax1.plot(df['Time_sec'], pressure_filtered, label='Filtered Pressure (Notch 11Hz)', 
             color='blue', linewidth=2)
    ax1.axhline(y=f0, color='green', linestyle=':', linewidth=2, label=f'F0 = {f0:.2f} kg')
    ax1.set_ylabel('Pressure (kg)', fontsize=12)
    ax1.set_title('Raw vs Filtered Pressure Comparison', fontsize=13, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.tick_params(labelsize=10)
    
    # 图2: 滤波差值
    ax2 = axes[1]
    ax2.plot(df['Time_sec'], filter_diff, label='Raw - Filtered', color='purple', linewidth=1.5)
    ax2.fill_between(df['Time_sec'], filter_diff, alpha=0.3, color='purple')
    ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax2.set_ylabel('Difference (kg)', fontsize=12)
    ax2.set_title('Filter Effect (Raw - Filtered)', fontsize=13, fontweight='bold')
    ax2.legend(loc='best', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.tick_params(labelsize=10)
    
    # 图3: 局部放大（寻找波动最大的区域）
    ax3 = axes[2]
    window_size = 50  # 1秒窗口
    rolling_std = pressure_raw.rolling(window=window_size).std()
    max_std_idx = rolling_std.idxmax()
    
    if max_std_idx > window_size and max_std_idx < len(df) - window_size:
        start_idx = max(0, max_std_idx - window_size)
        end_idx = min(len(df), max_std_idx + window_size)
        
        ax3.plot(df['Time_sec'].iloc[start_idx:end_idx], 
                pressure_raw.iloc[start_idx:end_idx], 
                label='Raw Pressure', color='red', linewidth=1.5, alpha=0.7, linestyle='--')
        ax3.plot(df['Time_sec'].iloc[start_idx:end_idx], 
                pressure_filtered.iloc[start_idx:end_idx], 
                label='Filtered Pressure', color='blue', linewidth=2)
        ax3.axhline(y=f0, color='green', linestyle=':', linewidth=2, label=f'F0 = {f0:.2f} kg')
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('Pressure (kg)', fontsize=12)
        ax3.set_title(f'Zoomed View - Peak Variation Region (t={df["Time_sec"].iloc[max_std_idx]:.2f}s)', 
                     fontsize=13, fontweight='bold')
        ax3.legend(loc='best', fontsize=10)
        ax3.grid(True, alpha=0.3)
        ax3.tick_params(labelsize=10)
    else:
        # 如果没有明显波动，显示最后2秒
        start_idx = max(0, len(df) - 100)
        ax3.plot(df['Time_sec'].iloc[start_idx:], 
                pressure_raw.iloc[start_idx:], 
                label='Raw Pressure', color='red', linewidth=1.5, alpha=0.7, linestyle='--')
        ax3.plot(df['Time_sec'].iloc[start_idx:], 
                pressure_filtered.iloc[start_idx:], 
                label='Filtered Pressure', color='blue', linewidth=2)
        ax3.axhline(y=f0, color='green', linestyle=':', linewidth=2, label=f'F0 = {f0:.2f} kg')
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('Pressure (kg)', fontsize=12)
        ax3.set_title('Zoomed View - Last 2 Seconds', fontsize=13, fontweight='bold')
        ax3.legend(loc='best', fontsize=10)
        ax3.grid(True, alpha=0.3)
        ax3.tick_params(labelsize=10)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 保存图片
    import os
    base_name = os.path.basename(csv_file).replace('.csv', '_pressure_filter_comparison.png')
    output_file = os.path.join(os.getcwd(), base_name)
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n对比图已保存: {output_file}")
    
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_pressure_filter_comparison.py <csv_file>")
        sys.exit(1)
    
    plot_pressure_comparison(sys.argv[1])
