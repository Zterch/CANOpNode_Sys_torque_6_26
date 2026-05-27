#!/usr/bin/env python3
"""
重力卸载算法数据分析脚本
分析电流控制效果和抖动原因
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def analyze_data(csv_file):
    # 读取CSV数据
    df = pd.read_csv(csv_file, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    
    print("=" * 80)
    print("重力卸载算法数据分析报告")
    print("=" * 80)
    
    # 1. 基本统计信息
    print("\n【1. 基本统计信息】")
    print(f"数据点数: {len(df)}")
    print(f"时间跨度: {df['Time'].iloc[0]} ~ {df['Time'].iloc[-1]}")
    
    # 2. 目标电流分析
    print("\n【2. 目标电流分析】")
    target_current = df['TargetCurrent(A)']
    print(f"目标电流 - 最小值: {target_current.min():.3f}A, 最大值: {target_current.max():.3f}A")
    print(f"目标电流 - 平均值: {target_current.mean():.3f}A, 标准差: {target_current.std():.6f}A")
    print(f"目标电流 - 唯一值数量: {target_current.nunique()}")
    
    if target_current.nunique() <= 5:
        print(f"⚠️ 警告: 目标电流几乎恒定! 实际值分布:")
        print(target_current.value_counts().sort_index())
    
    # 3. 实际电流分析
    print("\n【3. 实际电流分析】")
    actual_current = df['Current(A)']
    print(f"实际电流 - 最小值: {actual_current.min():.3f}A, 最大值: {actual_current.max():.3f}A")
    print(f"实际电流 - 平均值: {actual_current.mean():.3f}A, 标准差: {actual_current.std():.6f}A")
    
    # 4. 压力分析
    print("\n【4. 压力传感器分析】")
    pressure = df['Pressure(kg)']
    f0 = df['F0(kg)']
    deltaf = df['DeltaF']
    
    print(f"压力 - 最小值: {pressure.min():.3f}kg, 最大值: {pressure.max():.3f}kg")
    print(f"压力 - 平均值: {pressure.mean():.3f}kg, 标准差: {pressure.std():.3f}kg")
    print(f"F0(目标压力) - 平均值: {f0.mean():.3f}kg")
    print(f"DeltaF - 最小值: {deltaf.min():.3f}, 最大值: {deltaf.max():.3f}")
    print(f"DeltaF - 平均值: {deltaf.mean():.3f}, 标准差: {deltaf.std():.3f}")
    
    # 5. 电机速度分析
    print("\n【5. 电机速度分析】")
    motor_speed = df['MotorSpeed(rpm)']
    print(f"电机速度 - 最小值: {motor_speed.min():.1f}rpm, 最大值: {motor_speed.max():.1f}rpm")
    print(f"电机速度 - 平均值: {motor_speed.mean():.1f}rpm, 标准差: {motor_speed.std():.1f}rpm")
    print(f"电机速度变化范围: {motor_speed.max() - motor_speed.min():.1f}rpm")
    
    # 6. 绳子速度分析
    print("\n【6. 绳子速度分析】")
    rope_vel_raw = df['RopeVelocityRaw(m/s)']
    rope_vel_filt = df['RopeVelocityFiltered(m/s)']
    print(f"绳子原始速度 - 最小值: {rope_vel_raw.min():.4f}m/s, 最大值: {rope_vel_raw.max():.4f}m/s")
    print(f"绳子滤波速度 - 最小值: {rope_vel_filt.min():.4f}m/s, 最大值: {rope_vel_filt.max():.4f}m/s")
    print(f"绳子滤波速度 - 平均值: {rope_vel_filt.mean():.4f}m/s, 标准差: {rope_vel_filt.std():.4f}m/s")
    
    # 7. 抖动分析
    print("\n【7. 抖动分析】")
    # 计算电机速度的抖动（高频变化）
    motor_speed_diff = motor_speed.diff().abs()
    print(f"电机速度变化率 - 平均值: {motor_speed_diff.mean():.2f}rpm/20ms")
    print(f"电机速度变化率 - 最大值: {motor_speed_diff.max():.2f}rpm/20ms")
    
    # 计算压力抖动
    pressure_diff = pressure.diff().abs()
    print(f"压力变化率 - 平均值: {pressure_diff.mean():.3f}kg/20ms")
    print(f"压力变化率 - 最大值: {pressure_diff.max():.3f}kg/20ms")
    
    # 8. 电流控制效果分析
    print("\n【8. 电流控制效果分析】")
    current_error = (actual_current - target_current).abs()
    print(f"电流跟踪误差 - 平均值: {current_error.mean():.6f}A")
    print(f"电流跟踪误差 - 最大值: {current_error.max():.6f}A")
    
    # 9. 问题诊断
    print("\n" + "=" * 80)
    print("【问题诊断】")
    print("=" * 80)
    
    issues = []
    
    # 检查目标电流是否变化
    if target_current.nunique() <= 3:
        issues.append("❌ 目标电流几乎恒定(0.9A)，说明PI控制没有起作用!")
        issues.append("   原因: clutch_pi_integral可能为0，或者deltaf计算有问题")
    
    # 检查压力波动
    if pressure.std() > 1.0:
        issues.append(f"❌ 压力波动过大(标准差{pressure.std():.2f}kg)，系统不稳定!")
    
    # 检查电机速度抖动
    if motor_speed_diff.mean() > 5:
        issues.append(f"❌ 电机速度抖动严重(平均变化{motor_speed_diff.mean():.1f}rpm/20ms)!")
    
    # 检查DeltaF范围
    if deltaf.abs().max() < 0.5:
        issues.append("⚠️ DeltaF变化范围很小，可能摩擦力计算有问题")
    
    if not issues:
        print("✅ 未发现明显问题")
    else:
        for issue in issues:
            print(issue)
    
    # 10. 建议
    print("\n" + "=" * 80)
    print("【改进建议】")
    print("=" * 80)
    
    print("""
1. 目标电流恒定问题:
   - 检查 clutch_pi_integral 是否在累积
   - 检查 deltaf 的计算是否正确
   - 可能需要增大 CLUTCH_PI_KP 和 CLUTCH_PI_KI

2. 抖动问题:
   - 电机速度PID参数可能需要调整
   - 考虑增加速度滤波强度
   - 检查编码器数据质量

3. 压力波动:
   - 增加压力传感器滤波窗口
   - 检查机械系统是否有松动
    """)
    
    # 绘制图表
    print("\n正在生成分析图表...")
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('Gravity Unload Algorithm Analysis', fontsize=14)
    
    # 图1: 电流
    ax = axes[0, 0]
    ax.plot(df.index, df['Current(A)'], label='Actual Current', linewidth=1)
    ax.plot(df.index, df['TargetCurrent(A)'], label='Target Current', linewidth=1, linestyle='--')
    ax.set_ylabel('Current (A)')
    ax.set_title('Current Control')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 图2: 压力
    ax = axes[0, 1]
    ax.plot(df.index, df['Pressure(kg)'], label='Pressure', color='green', linewidth=1)
    ax.axhline(y=df['F0(kg)'].mean(), color='r', linestyle='--', label='F0 (Target)')
    ax.set_ylabel('Pressure (kg)')
    ax.set_title('Pressure vs F0')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 图3: DeltaF
    ax = axes[1, 0]
    ax.plot(df.index, df['DeltaF'], label='DeltaF', color='purple', linewidth=1)
    ax.axhline(y=0, color='r', linestyle='--', linewidth=0.5)
    ax.set_ylabel('DeltaF')
    ax.set_title('DeltaF (Friction Change)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 图4: 电机速度
    ax = axes[1, 1]
    ax.plot(df.index, df['MotorSpeed(rpm)'], label='Motor Speed', color='orange', linewidth=1)
    ax.set_ylabel('Speed (rpm)')
    ax.set_title('Motor Speed')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 图5: 绳子速度
    ax = axes[2, 0]
    ax.plot(df.index, df['RopeVelocityRaw(m/s)'], label='Raw', alpha=0.5, linewidth=0.8)
    ax.plot(df.index, df['RopeVelocityFiltered(m/s)'], label='Filtered', linewidth=1)
    ax.set_ylabel('Velocity (m/s)')
    ax.set_xlabel('Sample Index')
    ax.set_title('Rope Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 图6: 位置
    ax = axes[2, 1]
    ax.plot(df.index, df['RopeLength(m)'], label='Rope Length', color='brown', linewidth=1)
    ax.set_ylabel('Length (m)')
    ax.set_xlabel('Sample Index')
    ax.set_title('Rope Length (Position)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = csv_file.replace('.csv', '_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"图表已保存: {output_file}")
    plt.close()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_gravity_data.py <csv_file>")
        sys.exit(1)
    
    analyze_data(sys.argv[1])
