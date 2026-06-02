#!/usr/bin/env python3
"""
电流与压力关系局部放大分析脚本
详细显示力和电流的跳变关系，X轴间隔10ms
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def analyze_force_current_detail(csv_file):
    # 读取CSV数据
    df = pd.read_csv(csv_file, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    
    # 创建时间序列（秒）
    df['Time_sec'] = np.arange(len(df)) * 0.02  # 50Hz采样
    
    print("=" * 80)
    print("Force-Current Transient Relationship Analysis")
    print("=" * 80)
    
    # 提取关键数据
    pressure = df['PressureFiltered(kg)'] if 'PressureFiltered(kg)' in df.columns else df['Pressure(kg)']
    current = df['Current(A)'] * 1000  # 转换为mA
    
    # 找到电流跳变的时间段
    current_diff = np.diff(current)
    max_rise_idx = np.argmax(current_diff)
    
    # 以最快上升点为中心，前后各取15个点（约300ms，50Hz采样）
    zoom_center_idx = max_rise_idx
    zoom_start_idx = max(0, zoom_center_idx - 15)
    zoom_end_idx = min(len(df) - 1, zoom_center_idx + 20)
    zoom_start_time = df['Time_sec'].iloc[zoom_start_idx]
    zoom_end_time = df['Time_sec'].iloc[zoom_end_idx]
    
    # 提取放大区域的数据
    zoom_mask = (df['Time_sec'] >= zoom_start_time) & (df['Time_sec'] <= zoom_end_time)
    time_ms = (df['Time_sec'][zoom_mask] - zoom_start_time) * 1000  # 转换为毫秒
    pressure_zoom = pressure[zoom_mask].values
    current_zoom = current[zoom_mask].values
    
    # 计算延迟
    pressure_50 = pressure_zoom.min() + (pressure_zoom.max() - pressure_zoom.min()) * 0.5
    current_50 = current_zoom.min() + (current_zoom.max() - current_zoom.min()) * 0.5
    
    pressure_50_idx = np.argmin(np.abs(pressure_zoom - pressure_50))
    current_50_idx = np.argmin(np.abs(current_zoom - current_50))
    
    delay_ms = time_ms.values[current_50_idx] - time_ms.values[pressure_50_idx]
    
    print(f"\nAnalysis Results:")
    print(f"  Transient start time: {zoom_start_time:.3f}s")
    print(f"  Transient end time: {zoom_end_time:.3f}s")
    print(f"  Time window: {(zoom_end_time - zoom_start_time)*1000:.0f}ms")
    print(f"  Pressure 50% rise: {time_ms.values[pressure_50_idx]:.1f}ms")
    print(f"  Current 50% rise: {time_ms.values[current_50_idx]:.1f}ms")
    print(f"  Delay: {delay_ms:.1f}ms (Pressure -> Current)")
    
    # 创建详细分析图 - 只保留压力和电流关系
    fig, ax1 = plt.subplots(figsize=(14, 6))
    
    # 绘制压力和电流
    ax1.plot(time_ms, pressure_zoom, 'b-o', label='Pressure (Force)', linewidth=2.5, markersize=6)
    ax1_twin = ax1.twinx()
    ax1_twin.plot(time_ms, current_zoom, 'r--s', label='Current', linewidth=2.5, markersize=6)
    
    # 标记50%上升点
    ax1.axvline(x=time_ms.values[pressure_50_idx], color='blue', linestyle=':', linewidth=2, alpha=0.7)
    ax1_twin.axvline(x=time_ms.values[current_50_idx], color='red', linestyle=':', linewidth=2, alpha=0.7)
    
    # 添加延迟标注
    ax1.annotate('', xy=(time_ms.values[current_50_idx], pressure_zoom[pressure_50_idx]),
                xytext=(time_ms.values[pressure_50_idx], pressure_zoom[pressure_50_idx]),
                arrowprops=dict(arrowstyle='<->', color='green', lw=2.5))
    ax1.text((time_ms.values[pressure_50_idx] + time_ms.values[current_50_idx])/2, 
            pressure_zoom[pressure_50_idx] + 0.03,
            f'Delay: {delay_ms:.1f}ms', ha='center', fontsize=12, color='green', fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    # 设置标题和标签
    ax1.set_xlabel('Time (ms)', fontsize=13, fontweight='bold')
    ax1.set_ylabel('Pressure (kg)', color='blue', fontsize=13, fontweight='bold')
    ax1_twin.set_ylabel('Current (mA)', color='red', fontsize=13, fontweight='bold')
    ax1.set_title(f'Force-Current Transient Relationship (10ms resolution, {(zoom_end_time - zoom_start_time)*1000:.0f}ms window)', 
                  fontsize=14, fontweight='bold')
    
    # 设置X轴刻度为10ms间隔
    x_ticks = np.arange(0, time_ms.max() + 10, 10)
    ax1.set_xticks(x_ticks)
    ax1.set_xticklabels([f'{int(x)}' for x in x_ticks])
    
    # 添加网格线
    ax1.grid(True, alpha=0.4, linestyle=':', linewidth=0.8)
    ax1_twin.grid(False)
    
    # 图例
    ax1.legend(loc='upper left', fontsize=11)
    ax1_twin.legend(loc='upper right', fontsize=11)
    
    plt.tight_layout()
    
    # 保存图片
    import os
    base_name = os.path.basename(csv_file).replace('.csv', '')
    output_file = os.path.join(os.getcwd(), f'{base_name}_force_current_transient.png')
    plt.savefig(output_file, dpi=200, bbox_inches='tight')
    print(f"\nChart saved: {output_file}")
    plt.close()
    
    # 生成数据表格
    print("\n" + "=" * 80)
    print("Detailed Data Points:")
    print("=" * 80)
    print(f"{'Time(ms)':<12} {'Pressure(kg)':<15} {'Current(mA)':<15}")
    print("-" * 80)
    for i in range(len(time_ms)):
        print(f"{time_ms.values[i]:<12.1f} {pressure_zoom[i]:<15.3f} {current_zoom[i]:<15.1f}")
    
    print("\n" + "=" * 80)
    print("Analysis Complete!")
    print("=" * 80)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_force_current_detail.py <csv_file>")
        sys.exit(1)
    
    analyze_force_current_detail(sys.argv[1])
