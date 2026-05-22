#!/usr/bin/env python3
"""
四合一曲线图 - 在一个PNG中包含：
1. 合并图（压力缩放、转速偏移、电流缩放，共零点）
2. 压力单独图
3. 电机转速单独图
4. 电流单独图（实际值与命令值）
图片保存到数据文件所在目录的上一级目录
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

# ========== 用户可调参数 ==========
PRESSURE_SCALE = 20.0   # 压力偏移放大倍数
CURRENT_SCALE  = 20.0   # 电流偏移放大倍数
TARGET_SPEED   = -30.0   # 目标稳定转速 (rpm)
# =================================

def plot_all_in_one(csv_file, output_dir, base_name):
    df = pd.read_csv(csv_file)
    df.columns = df.columns.str.strip()
    df['Time'] = df['Time'].str.strip()
    df['Time'] = pd.to_datetime(df['Time'], format='%H:%M:%S.%f')
    start_time = df['Time'].iloc[0]
    df['Time_sec'] = (df['Time'] - start_time).dt.total_seconds()

    pressure0 = df['Pressure(kg)'].iloc[0]

    # 计算偏移缩放
    df['Pressure_scaled'] = (df['Pressure(kg)'] - pressure0) * PRESSURE_SCALE
    df['Speed_scaled'] = df['MotorSpeed(rpm)'] - TARGET_SPEED
    df['Current_scaled'] = df['Current(A)'] * CURRENT_SCALE
    df['TargetCurrent_scaled'] = df['TargetCurrent(A)'] * CURRENT_SCALE

    # 创建 2x2 子图
    fig = plt.figure(figsize=(16, 12))
    # 使用 gridspec 调节子图间距
    gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)

    # 子图1: 合并图 (左上)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df['Time_sec'], df['Pressure_scaled'], 'g-', lw=1.5,
             label=f'Pressure (Δ×{PRESSURE_SCALE:.0f}) [base={pressure0:.2f} kg]')
    ax1.plot(df['Time_sec'], df['Speed_scaled'], 'b-', lw=1.5,
             label=f'Motor Speed offset (rpm) [base={TARGET_SPEED} rpm]')
    ax1.plot(df['Time_sec'], df['Current_scaled'], 'r-', lw=1.5,
             label=f'Current Actual (×{CURRENT_SCALE:.0f})')
    ax1.plot(df['Time_sec'], df['TargetCurrent_scaled'], 'm--', lw=1.5,
             label=f'Current Command (×{CURRENT_SCALE:.0f})')
    ax1.axhline(0, color='gray', ls='--', lw=0.8, alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Scaled Offset')
    ax1.set_title('Combined (Zero-Aligned & Scaled)', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=8)
    # 添加参数文本框
    info = f'P scale:{PRESSURE_SCALE:.0f}x\nC scale:{CURRENT_SCALE:.0f}x\nTarget rpm:{TARGET_SPEED}'
    ax1.text(0.02, 0.98, info, transform=ax1.transAxes, fontsize=8, va='top',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # 子图2: 压力单独 (右上)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(df['Time_sec'], df['Pressure(kg)'], 'g-', lw=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Pressure (kg)')
    ax2.set_title('Pressure Sensor', fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(pressure0, color='gray', ls='--', lw=0.8, alpha=0.5, label=f'Initial: {pressure0:.2f} kg')
    ax2.legend(loc='best', fontsize=8)

    # 子图3: 转速单独 (左下)
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(df['Time_sec'], df['MotorSpeed(rpm)'], 'b-', lw=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Motor Speed (rpm)')
    ax3.set_title('Motor Speed', fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(TARGET_SPEED, color='gray', ls='--', lw=0.8, alpha=0.5, label=f'Target: {TARGET_SPEED} rpm')
    ax3.legend(loc='best', fontsize=8)

    # 子图4: 电流单独 (右下)
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(df['Time_sec'], df['Current(A)'], 'r-', lw=1.5, label='Actual')
    ax4.plot(df['Time_sec'], df['TargetCurrent(A)'], 'm--', lw=1.5, label='Command')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Current (A)')
    ax4.set_title('Current (Actual vs Command)', fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend(loc='best', fontsize=8)

    # 整体标题
    fig.suptitle(f'Data Analysis: {os.path.basename(csv_file)}', fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # 保存
    output_path = os.path.join(output_dir, f"{base_name}_all_plots.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"All-in-one plot saved to: {output_path}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_curves.py <csv_file>")
        sys.exit(1)
    csv_file = sys.argv[1]
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)

    csv_dir = os.path.dirname(csv_file)
    parent_dir = os.path.dirname(csv_dir)
    os.makedirs(parent_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(csv_file))[0]

    plot_all_in_one(csv_file, parent_dir, base_name)

if __name__ == '__main__':
    main()