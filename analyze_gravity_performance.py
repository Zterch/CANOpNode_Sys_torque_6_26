#!/usr/bin/env python3
"""
重力卸载算法性能分析脚本
展示算法效果和恒力控制性能
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
from matplotlib.gridspec import GridSpec

def analyze_gravity_performance(csv_file, sample_period_ms=10):
    # 读取CSV数据
    df = pd.read_csv(csv_file, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    
    # 创建时间序列（秒）
    # 默认使用10ms周期（100Hz），可通过参数调整
    sample_period_s = sample_period_ms / 1000.0
    df['Time_sec'] = np.arange(len(df)) * sample_period_s
    
    print("=" * 80)
    print("重力卸载算法性能分析报告")
    print("=" * 80)
    
    # 1. 基本统计信息
    print("\n【1. 基本统计信息】")
    print(f"数据点数: {len(df)}")
    print(f"时间跨度: {df['Time_sec'].iloc[-1]:.1f} 秒")
    print(f"采样频率: {int(1/sample_period_s)} Hz")
    print(f"采样周期: {sample_period_ms} ms")
    
    # 2. 压力控制性能分析
    print("\n【2. 压力控制性能分析】")
    
    # 检查是否有原始和滤波后的压力数据（新格式）
    if 'PressureRaw(kg)' in df.columns and 'PressureFiltered(kg)' in df.columns:
        pressure_raw = df['PressureRaw(kg)']
        pressure_filtered = df['PressureFiltered(kg)']
        pressure = pressure_filtered  # 使用滤波后的值作为主要压力数据
        has_raw_filtered = True
        print("  使用新的CSV格式（包含原始和滤波后压力值）")
    else:
        pressure = df['Pressure(kg)']
        pressure_raw = None
        pressure_filtered = None
        has_raw_filtered = False
        print("  使用旧的CSV格式（仅压力值）")
    
    # 检查是否有新的重量采集数据列（新增UART/TTL重量采集模块）
    if 'WeightRaw(kg)' in df.columns and 'WeightFiltered(kg)' in df.columns:
        weight_raw = df['WeightRaw(kg)']
        weight_filtered = df['WeightFiltered(kg)']
        has_weight_data = True
        print("  检测到新的重量采集数据（UART/TTL模块，100Hz）")
    else:
        weight_raw = None
        weight_filtered = None
        has_weight_data = False
    
    f0 = df['F0(kg)'].iloc[0]  # 目标压力
    delta_f = df['DeltaF']
    
    print(f"目标压力 (F0): {f0:.3f} kg")
    
    if has_raw_filtered:
        print(f"\n原始压力值:")
        print(f"  最小值: {pressure_raw.min():.3f} kg, 最大值: {pressure_raw.max():.3f} kg")
        print(f"  平均值: {pressure_raw.mean():.3f} kg, 标准差: {pressure_raw.std():.3f} kg")
        print(f"  变化范围: {pressure_raw.max() - pressure_raw.min():.3f} kg")
        
        print(f"\n滤波后压力值:")
        print(f"  最小值: {pressure_filtered.min():.3f} kg, 最大值: {pressure_filtered.max():.3f} kg")
        print(f"  平均值: {pressure_filtered.mean():.3f} kg, 标准差: {pressure_filtered.std():.3f} kg")
        print(f"  变化范围: {pressure_filtered.max() - pressure_filtered.min():.3f} kg")
        
        # 计算滤波效果
        filter_diff = pressure_raw - pressure_filtered
        print(f"\n滤波效果:")
        print(f"  标准差降低: {(1 - pressure_filtered.std()/pressure_raw.std())*100:.1f}%")
        print(f"  峰峰值降低: {(1 - (pressure_filtered.max()-pressure_filtered.min())/(pressure_raw.max()-pressure_raw.min()))*100:.1f}%")
    else:
        print(f"实际压力 - 最小值: {pressure.min():.3f} kg, 最大值: {pressure.max():.3f} kg")
        print(f"实际压力 - 平均值: {pressure.mean():.3f} kg, 标准差: {pressure.std():.3f} kg")
        print(f"压力变化范围: {pressure.max() - pressure.min():.3f} kg")
    
    # 显示重量采集数据统计（新增）
    if has_weight_data:
        print(f"\n【2.1 重量采集模块数据（UART/TTL, 100Hz）】")
        print(f"原始重量值:")
        print(f"  最小值: {weight_raw.min():.3f} kg, 最大值: {weight_raw.max():.3f} kg")
        print(f"  平均值: {weight_raw.mean():.3f} kg, 标准差: {weight_raw.std():.3f} kg")
        print(f"  变化范围: {weight_raw.max() - weight_raw.min():.3f} kg")
        
        print(f"\n滤波后重量值:")
        print(f"  最小值: {weight_filtered.min():.3f} kg, 最大值: {weight_filtered.max():.3f} kg")
        print(f"  平均值: {weight_filtered.mean():.3f} kg, 标准差: {weight_filtered.std():.3f} kg")
        print(f"  变化范围: {weight_filtered.max() - weight_filtered.min():.3f} kg")
    
    # 计算稳态误差
    steady_state_start = int(len(df) * 0.3)  # 后70%作为稳态
    steady_pressure = pressure.iloc[steady_state_start:]
    steady_error = steady_pressure - f0
    print(f"\n稳态压力偏差:")
    print(f"  平均值: {steady_error.mean():.3f} kg")
    print(f"  标准差: {steady_error.std():.3f} kg")
    print(f"  最大偏差: {steady_error.abs().max():.3f} kg")
    
    # 从CSV读取真实的PID控制项数据
    # 如果CSV中有PI_P(mA)和PI_I(mA)列，使用真实数据
    if 'PI_P(mA)' in df.columns and 'PI_I(mA)' in df.columns:
        # 使用采集的真实数据
        p_term = df['PI_P(mA)']
        i_term = df['PI_I(mA)']
        # 读取D项（如果存在）
        if 'PI_D(mA)' in df.columns:
            d_term = df['PI_D(mA)']
            has_d_term = True
            pi_output = p_term + i_term + d_term
        else:
            d_term = None
            has_d_term = False
            pi_output = p_term + i_term
        using_real_data = True

        # 读取PI累积电流（last_current_mA）
        if 'PI_LastCurrent(mA)' in df.columns:
            pi_last_current = df['PI_LastCurrent(mA)']
            has_last_current = True
        else:
            pi_last_current = None
            has_last_current = False

        print(f"\n【PID控制分析 - 使用真实采集数据】")
        print(f"P项范围: {p_term.min():.2f} ~ {p_term.max():.2f} mA")
        print(f"I项范围: {i_term.min():.2f} ~ {i_term.max():.2f} mA")
        if has_d_term:
            print(f"D项范围: {d_term.min():.2f} ~ {d_term.max():.2f} mA")
        print(f"PID输出范围: {pi_output.min():.2f} ~ {pi_output.max():.2f} mA")
        if has_last_current:
            print(f"PID累积电流(LastCurrent)范围: {pi_last_current.min():.2f} ~ {pi_last_current.max():.2f} mA")
    else:
        # 从配置文件中读取参数并计算
        import re
        config_file = '/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/algorithms/algorithm_config.h'
        
        def read_config_value(filename, pattern):
            """从配置文件读取宏定义值"""
            try:
                with open(filename, 'r') as f:
                    content = f.read()
                    match = re.search(pattern, content)
                    if match:
                        return float(match.group(1))
            except Exception as e:
                print(f"Warning: Could not read config: {e}")
            return None
        
        # 读取配置值
        KP = read_config_value(config_file, r'CLUTCH_PI_KP\s+(\d+\.?\d*)f?')
        KI = read_config_value(config_file, r'CLUTCH_PI_KI\s+(\d+\.?\d*)f?')
        DT = sample_period_s   # 使用实际采样周期（默认10ms = 0.01s）
        
        # 如果读取失败，使用默认值
        if KP is None:
            KP = 40.0
            print(f"Warning: Could not read KP from config, using default: {KP}")
        if KI is None:
            KI = 10.0
            print(f"Warning: Could not read KI from config, using default: {KI}")
        
        # 计算P项（比例项）
        p_term = KP * delta_f
        
        # 计算I项（积分项）- 通过累加得到
        i_term = np.zeros(len(df))
        integral = 0.0
        for i in range(len(df)):
            integral += KI * delta_f.iloc[i] * DT
            # 积分限幅
            integral = np.clip(integral, -100.0, 100.0)
            i_term[i] = integral
        
        # PI总输出
        pi_output = p_term + i_term
        using_real_data = False
        
        print(f"\n【PI控制分析 - 使用计算数据（CSV中没有PI列）】")
        print(f"Kp = {KP}, Ki = {KI}")
        print(f"P项范围: {p_term.min():.2f} ~ {p_term.max():.2f} mA")
        print(f"I项范围: {i_term.min():.2f} ~ {i_term.max():.2f} mA")
        print(f"PI输出范围: {pi_output.min():.2f} ~ {pi_output.max():.2f} mA")
    
    # 3. 电流控制性能分析
    print("\n【3. 电流控制性能分析】")
    current = df['Current(A)'] * 1000  # 转换为mA
    target_current = df['TargetCurrent(A)'] * 1000
    
    print(f"实际电流 - 最小值: {current.min():.1f} mA, 最大值: {current.max():.1f} mA")
    print(f"实际电流 - 平均值: {current.mean():.1f} mA, 标准差: {current.std():.1f} mA")
    print(f"目标电流 - 最小值: {target_current.min():.1f} mA, 最大值: {target_current.max():.1f} mA")
    
    # 电流跟踪误差
    current_error = (current - target_current).abs()
    print(f"\n电流跟踪误差:")
    print(f"  平均值: {current_error.mean():.2f} mA")
    print(f"  最大值: {current_error.max():.2f} mA")
    
    # 4. 电机速度分析
    print("\n【4. 电机速度分析】")
    motor_speed = df['MotorSpeed(rpm)']
    print(f"电机速度 - 最小值: {motor_speed.min():.1f} rpm, 最大值: {motor_speed.max():.1f} rpm")
    print(f"电机速度 - 平均值: {motor_speed.mean():.1f} rpm, 标准差: {motor_speed.std():.1f} rpm")
    print(f"电机速度变化范围: {motor_speed.max() - motor_speed.min():.1f} rpm")
    
    # 5. 位置变化分析
    print("\n【5. 位置变化分析】")
    position = df['RopeLength(m)'] * 1000  # 转换为mm
    print(f"绳子长度变化: {position.min():.2f} mm -> {position.max():.2f} mm")
    print(f"总位移: {position.max() - position.min():.2f} mm")
    
    # 6. 关键时间点检测
    print("\n【6. 关键时间点检测】")
    
    # 找到重量添加时间点（压力显著增加）
    pressure_diff = pressure.diff()
    significant_change_threshold = 0.05  # 压力变化阈值 kg
    significant_change_idx = np.where(pressure_diff > significant_change_threshold)[0]
    
    weight_add_time = None
    stable_time = None
    settling_time = None
    
    if len(significant_change_idx) > 0:
        weight_add_idx = significant_change_idx[0]
        weight_add_time = df['Time_sec'].iloc[weight_add_idx]
        print(f"重量添加时间点: {weight_add_time:.2f} 秒 (压力突增)")
        
        # 找到稳定时间点（DeltaF首次回零）
        # 从重量添加后开始寻找DeltaF首次接近零的点
        for i in range(weight_add_idx + 10, len(df)):  # 至少0.2秒后
            if abs(delta_f.iloc[i]) < 0.02:  # DeltaF接近零（±20g）
                stable_idx = i
                stable_time = df['Time_sec'].iloc[stable_idx]
                settling_time = stable_time - weight_add_time
                print(f"控制稳定时间点: {stable_time:.2f} 秒 (DeltaF首次回零)")
                print(f"调节时间: {settling_time:.2f} 秒")
                break
        
        if stable_time is None:
            print("未检测到稳定点（DeltaF未回零）")
    else:
        print("未检测到重量添加事件")
    
    # 7. 恒力控制效果评估
    print("\n【7. 恒力控制效果评估】")
    
    # 计算压力超调量
    pressure_overshoot = (pressure.max() - f0) / f0 * 100 if f0 > 0 else 0
    print(f"压力超调量: {pressure_overshoot:.2f}%")
    
    # 计算调节时间（压力进入±5%范围的时间）
    tolerance = 0.5  # kg
    within_tolerance = np.abs(pressure - f0) < tolerance
    if np.any(within_tolerance):
        settling_idx = np.where(within_tolerance)[0][0]
        settling_time = df['Time_sec'].iloc[settling_idx]
        print(f"调节时间 (±{tolerance}kg): {settling_time:.2f} 秒")
    
    # 绘制图表
    print("\n正在生成分析图表...")
    
    # 创建第一个图表 - 总览图
    fig = plt.figure(figsize=(18, 22))
    gs = GridSpec(9, 2, figure=fig, hspace=0.35, wspace=0.3)
    
    # 图1: 压力控制效果（带关键时间点标记）
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(df['Time_sec'], pressure, label='Filtered Pressure', color='blue', linewidth=2)
    ax1.axhline(y=f0, color='r', linestyle='--', linewidth=2, label=f'Target (F0={f0:.2f}kg)')
    ax1.fill_between(df['Time_sec'], f0 - 0.5, f0 + 0.5, alpha=0.2, color='green', label='Tolerance (±0.5kg)')
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax1.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, label=f'Weight Added ({weight_add_time:.2f}s)')
    if stable_time is not None and settling_time is not None:
        ax1.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, label=f'Stable dF=0 ({stable_time:.2f}s, dt={settling_time:.2f}s)')
    
    ax1.set_ylabel('Pressure (kg)')
    ax1.set_title('Pressure Control Performance - Gravity Unload Algorithm', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([f0 - 1, pressure.max() + 0.5])
    
    # 图1b: 原始压力vs滤波后压力对比（新增）
    if has_raw_filtered:
        ax1b = fig.add_subplot(gs[1, :])
        ax1b.plot(df['Time_sec'], pressure_raw, label='Raw Pressure', color='red', 
                 linewidth=1.5, alpha=0.7, linestyle='--')
        ax1b.plot(df['Time_sec'], pressure_filtered, label='Filtered Pressure (Notch 11Hz)', 
                 color='blue', linewidth=2)
        ax1b.axhline(y=f0, color='green', linestyle=':', linewidth=2, label=f'F0 = {f0:.2f} kg')
        
        # 标记关键时间点
        if weight_add_time is not None:
            ax1b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
        if stable_time is not None:
            ax1b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
        
        ax1b.set_ylabel('Pressure (kg)')
        ax1b.set_title('Raw vs Filtered Pressure Comparison', fontsize=13, fontweight='bold')
        ax1b.legend(loc='best', fontsize=10)
        ax1b.grid(True, alpha=0.3)
    
    # 图1c: 重量采集模块数据（新增）
    if has_weight_data:
        ax1c = fig.add_subplot(gs[2, :])
        ax1c.plot(df['Time_sec'], weight_raw, label='Weight Raw (UART/TTL)', color='brown', 
                 linewidth=1.5, alpha=0.7, linestyle='--')
        ax1c.plot(df['Time_sec'], weight_filtered, label='Weight Filtered (100Hz)', 
                 color='darkgreen', linewidth=2)
        
        # 标记关键时间点
        if weight_add_time is not None:
            ax1c.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
        if stable_time is not None:
            ax1c.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
        
        ax1c.set_ylabel('Weight (kg)')
        ax1c.set_title('Weight Sensor Data (New UART/TTL Module, 100Hz)', fontsize=13, fontweight='bold')
        ax1c.legend(loc='best', fontsize=10)
        ax1c.grid(True, alpha=0.3)
    
    # 图2: DeltaF + 前馈电流 + AlgoDeltaF对比（双Y轴）
    ax2 = fig.add_subplot(gs[2, 0])
    
    # 左Y轴：DeltaF（数据采集计算）和 AlgoDeltaF（算法实际使用）
    ax2.plot(df['Time_sec'], delta_f, label='DeltaF (Data Collection)', color='purple', linewidth=1.5)
    if 'AlgoDeltaF(kg)' in df.columns:
        ax2.plot(df['Time_sec'], df['AlgoDeltaF(kg)'], label='AlgoDeltaF (Algorithm Used)', 
                color='red', linewidth=1.5, linestyle='--', alpha=0.7)
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax2.set_ylabel('DeltaF (kg)', color='purple')
    ax2.tick_params(axis='y', labelcolor='purple')
    
    # 右Y轴：前馈电流
    if 'Feedforward(mA)' in df.columns:
        ax2_twin = ax2.twinx()
        ax2_twin.plot(df['Time_sec'], df['Feedforward(mA)'], label='Feedforward Current', 
                     color='orange', linewidth=2, linestyle='-.')
        ax2_twin.set_ylabel('Feedforward Current (mA)', color='orange')
        ax2_twin.tick_params(axis='y', labelcolor='orange')
        ax2_twin.legend(loc='upper right')
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax2.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax2.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    
    ax2.set_title('DeltaF vs Feedforward Current (Red=AlgoDeltaF, Purple=DataDeltaF)')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)
    
    # 图3: PI控制项分解 (缩小放在总览图中)
    ax3 = fig.add_subplot(gs[2, 1])
    if using_real_data:
        ax3.plot(df['Time_sec'], p_term, label='P Term (Real Data)', color='orange', linewidth=1.5)
        ax3.plot(df['Time_sec'], i_term, label='I Term (Real Data)', color='cyan', linewidth=1.5)
    else:
        ax3.plot(df['Time_sec'], p_term, label=f'P Term (Kp={KP})', color='orange', linewidth=1.5)
        ax3.plot(df['Time_sec'], i_term, label=f'I Term (Ki={KI})', color='cyan', linewidth=1.5)
    ax3.plot(df['Time_sec'], pi_output, label='PI Total Output', color='red', linewidth=2, linestyle='--')
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax3.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax3.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax3.set_ylabel('Current Adjustment (mA)')
    ax3.set_title('PI Control Components (P & I Terms)')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # 图4: 电流控制
    ax4 = fig.add_subplot(gs[3, 0])
    ax4.plot(df['Time_sec'], current, label='Actual Current', color='orange', linewidth=1.5)
    ax4.plot(df['Time_sec'], target_current, label='Target Current', color='red', linewidth=1.5, linestyle='--')
    ax4.axhline(y=50, color='gray', linestyle=':', linewidth=1, alpha=0.5, label='Base Current (50mA)')
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax4.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax4.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax4.set_ylabel('Current (mA)')
    ax4.set_title('Current Control')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 图5: PI输出与电流关系 (新增！)
    ax5 = fig.add_subplot(gs[3, 1])
    ax5.plot(df['Time_sec'], pi_output, label='PI Output', color='blue', linewidth=1.5)
    ax5_twin = ax5.twinx()
    ax5_twin.plot(df['Time_sec'], current - 50, label='Current - Base (50mA)', color='red', linewidth=1.5, linestyle='--')
    ax5.set_ylabel('PI Output (mA)', color='blue')
    ax5_twin.set_ylabel('Actual Current Adjustment (mA)', color='red')
    ax5.set_title('PI Output vs Actual Current Adjustment')
    ax5.legend(loc='upper left')
    ax5_twin.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)
    
    # 图6: 电机速度
    ax6 = fig.add_subplot(gs[4, 0])
    ax6.plot(df['Time_sec'], motor_speed, label='Motor Speed', color='green', linewidth=1.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax6.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax6.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax6.set_ylabel('Speed (rpm)')
    ax6.set_title('Motor Speed Response')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # 图7: 位置变化
    ax7 = fig.add_subplot(gs[4, 1])
    ax7.plot(df['Time_sec'], position, label='Rope Length', color='brown', linewidth=1.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax7.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax7.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax7.set_ylabel('Length (mm)')
    ax7.set_title('Rope Length (Position)')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # 图8: 绳子速度
    ax8 = fig.add_subplot(gs[5, 0])
    ax8.plot(df['Time_sec'], df['RopeVelocityRaw(m/s)'], label='Raw', alpha=0.5, linewidth=0.8)
    ax8.plot(df['Time_sec'], df['RopeVelocityFiltered(m/s)'], label='Filtered', linewidth=1.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax8.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax8.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax8.set_ylabel('Velocity (m/s)')
    ax8.set_xlabel('Time (s)')
    ax8.set_title('Rope Velocity')
    ax8.legend()
    ax8.grid(True, alpha=0.3)
    
    # 图8b: 力传感器（压力）和实际电流的关系 - 电流变化段局部放大（新增）
    ax8b = fig.add_subplot(gs[5, 1])
    
    # 找到电流快速上升的时间段（更精确的检测）
    current_change_threshold = (current.max() - current.min()) * 0.05  # 5%阈值，更敏感
    current_change_mask = (current - current.min()) > current_change_threshold
    change_indices = np.where(current_change_mask)[0]
    
    if len(change_indices) > 0:
        # 找到电流快速上升的起始点（斜率最大的区域）
        current_diff = np.diff(current)
        max_rise_idx = np.argmax(current_diff)  # 找到电流上升最快的点
        
        # 以最快上升点为中心，前后各取15个点（约300ms，50Hz采样）
        zoom_center_idx = max_rise_idx
        zoom_start_idx = max(0, zoom_center_idx - 15)
        zoom_end_idx = min(len(df) - 1, zoom_center_idx + 15)
        zoom_start_time = df['Time_sec'].iloc[zoom_start_idx]
        zoom_end_idx = min(len(df) - 1, zoom_center_idx + 25)
        zoom_end_time = df['Time_sec'].iloc[zoom_end_idx]
        
        # 局部放大显示
        zoom_mask = (df['Time_sec'] >= zoom_start_time) & (df['Time_sec'] <= zoom_end_time)
        time_ms = (df['Time_sec'][zoom_mask] - zoom_start_time) * 1000  # 转换为毫秒，从0开始
        
        ax8b.plot(time_ms, pressure[zoom_mask], label='Pressure (Force)', color='blue', linewidth=2.5, marker='o', markersize=4)
        ax8b_twin = ax8b.twinx()
        ax8b_twin.plot(time_ms, current[zoom_mask], label='Actual Current', color='red', linewidth=2.5, linestyle='--', marker='s', markersize=4)
        
        # 设置标题显示放大区间（精确到毫秒）
        duration_ms = (zoom_end_time - zoom_start_time) * 1000
        ax8b.set_title(f'Pressure vs Current (ZOOM: {duration_ms:.0f}ms window, {zoom_start_time:.3f}s start)', 
                      fontsize=10, fontweight='bold')
        
        # X轴显示毫秒
        ax8b.set_xlabel('Time (ms)', fontsize=11)
        
        # 标记重量添加时间点（转换为相对毫秒）
        if weight_add_time is not None and zoom_start_time <= weight_add_time <= zoom_end_time:
            weight_add_ms = (weight_add_time - zoom_start_time) * 1000
            ax8b.axvline(x=weight_add_ms, color='orange', linestyle='--', linewidth=2, alpha=0.8, label='Weight Added')
            
        # 添加网格线便于读数
        ax8b.grid(True, alpha=0.4, linestyle=':')
        ax8b_twin.grid(False)
        
        # 计算并显示延迟
        pressure_zoom = pressure[zoom_mask].values
        current_zoom = current[zoom_mask].values
        time_zoom_ms = time_ms.values
        
        # 找到压力上升50%和电流上升50%的时间点
        pressure_50 = pressure_zoom.min() + (pressure_zoom.max() - pressure_zoom.min()) * 0.5
        current_50 = current_zoom.min() + (current_zoom.max() - current_zoom.min()) * 0.5
        
        pressure_50_idx = np.argmin(np.abs(pressure_zoom - pressure_50))
        current_50_idx = np.argmin(np.abs(current_zoom - current_50))
        
        delay_ms = time_zoom_ms[current_50_idx] - time_zoom_ms[pressure_50_idx]
        
        # 在图上标注延迟
        ax8b.text(0.05, 0.95, f'Delay: {delay_ms:.1f}ms\n(Pressure→Current 50% rise)', 
                 transform=ax8b.transAxes, fontsize=9, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        
        # 标记50%上升点
        ax8b.axvline(x=time_zoom_ms[pressure_50_idx], color='blue', linestyle=':', linewidth=1.5, alpha=0.6)
        ax8b_twin.axvline(x=time_zoom_ms[current_50_idx], color='red', linestyle=':', linewidth=1.5, alpha=0.6)
        
    else:
        # 如果没有明显的电流变化，显示全图
        ax8b.plot(df['Time_sec'], pressure, label='Pressure (Force)', color='blue', linewidth=2)
        ax8b_twin = ax8b.twinx()
        ax8b_twin.plot(df['Time_sec'], current, label='Actual Current', color='red', linewidth=2, linestyle='--')
        ax8b.set_title('Pressure vs Current Relationship (Full View)')
        ax8b.set_xlabel('Time (s)')
        
        # 标记关键时间点
        if weight_add_time is not None:
            ax8b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    
    ax8b.set_ylabel('Pressure (kg)', color='blue')
    ax8b_twin.set_ylabel('Current (mA)', color='red')
    ax8b.legend(loc='upper left')
    ax8b_twin.legend(loc='upper right')
    ax8b.grid(True, alpha=0.3)
    
    # 图9: 压力误差分析
    ax9 = fig.add_subplot(gs[6, 0])
    pressure_error = pressure - f0
    ax9.plot(df['Time_sec'], pressure_error, label='Pressure Error', color='red', linewidth=1.5)
    ax9.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax9.axhline(y=0.5, color='g', linestyle='--', linewidth=1, alpha=0.5)
    ax9.axhline(y=-0.5, color='g', linestyle='--', linewidth=1, alpha=0.5)
    ax9.fill_between(df['Time_sec'], -0.5, 0.5, alpha=0.1, color='green')
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax9.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax9.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax9.set_ylabel('Error (kg)')
    ax9.set_xlabel('Time (s)')
    ax9.set_title('Pressure Control Error')
    ax9.legend()
    ax9.grid(True, alpha=0.3)
    
    # 图9b: 电流误差分析（新增，填充右侧）
    ax9b = fig.add_subplot(gs[6, 1])
    current_error = target_current - current
    ax9b.plot(df['Time_sec'], current_error, label='Current Error (Target-Actual)', color='purple', linewidth=1.5)
    ax9b.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax9b.axhline(y=current_error.mean(), color='blue', linestyle='--', linewidth=1, alpha=0.7, label=f'Mean: {current_error.mean():.1f}mA')
    ax9b.fill_between(df['Time_sec'], current_error, alpha=0.3, color='purple')
    ax9b.set_ylabel('Current Error (mA)')
    ax9b.set_xlabel('Time (s)')
    ax9b.set_title('Current Tracking Error')
    ax9b.legend()
    ax9b.grid(True, alpha=0.3)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax9b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax9b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)

    # 图10: 实际电流（新增）- 放大纵坐标以显示细微变化
    ax10 = fig.add_subplot(gs[7, :])
    ax10.plot(df['Time_sec'], current, label='Actual Current', color='darkblue', linewidth=2)
    ax10.axhline(y=50, color='gray', linestyle=':', linewidth=1, alpha=0.7, label='Base Current (50mA)')
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax10.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, label=f'Weight Added ({weight_add_time:.2f}s)')
    if stable_time is not None:
        ax10.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, label=f'Stable ({stable_time:.2f}s)')
    
    ax10.set_ylabel('Current (mA)')
    ax10.set_xlabel('Time (s)')
    ax10.set_title('Actual Current Only (Zoomed)', fontsize=14, fontweight='bold')
    # 自动调整Y轴范围以放大电流变化
    current_min = current.min()
    current_max = current.max()
    current_range = current_max - current_min
    if current_range < 10:  # 如果变化小于10mA，添加padding
        y_margin = max(2.0, current_range * 0.2)
    else:
        y_margin = current_range * 0.1
    ax10.set_ylim([current_min - y_margin, current_max + y_margin])
    ax10.legend(loc='upper right')
    ax10.grid(True, alpha=0.3)
    
    # 图11: 关键时间点总结（新增）- 使用英文避免乱码
    ax11 = fig.add_subplot(gs[8, :])
    ax11.axis('off')
    
    if weight_add_time is not None and stable_time is not None and settling_time is not None:
        summary_text = f"""Key Timing Summary
{'='*60}
Weight Added Time:     {weight_add_time:.2f} s
Stable Time (dF=0):    {stable_time:.2f} s
Settling Time:         {settling_time:.2f} s
Pressure Overshoot:    {pressure_overshoot:.2f}%
Steady State Error:    {steady_error.mean():.3f} +/- {steady_error.std():.3f} kg
{'='*60}"""
    elif weight_add_time is not None:
        summary_text = f"""Key Timing Summary
{'='*60}
Weight Added Time:     {weight_add_time:.2f} s
Stable Time:           Not detected (dF did not return to zero)
{'='*60}"""
    else:
        summary_text = """Key Timing Summary
{'='*60}
Weight added event not detected
{'='*60}"""
    
    ax11.text(0.5, 0.5, summary_text, transform=ax11.transAxes, fontsize=11,
              verticalalignment='center', horizontalalignment='center',
              fontfamily='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    # 将输出文件保存到当前目录，避免权限问题
    import os
    base_name = os.path.basename(csv_file).replace('.csv', '_performance_analysis.png')
    output_file = os.path.join(os.getcwd(), base_name)
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n总览图表已保存: {output_file}")
    plt.close()
    
    # 创建第二个图表 - PID控制项详细图（整合力-电流关系和PID分量）
    # 子图布局：
    # - 子图1: 增量式PID三个分量(P、I、D) + 总PID输出 + 前馈量 放在同一个图中
    # - 子图2: 力（压力）和电流的关系
    fig2, axes = plt.subplots(2, 1, figsize=(16, 14))
    fig2.suptitle('PID Control Components & Force-Current Relationship', fontsize=16, fontweight='bold')

    # 子图1: 增量式PID三个分量 + 总PID输出 + 前馈量
    ax_pid_all = axes[0]
    
    # P项（比例项）
    ax_pid_all.plot(df['Time_sec'], p_term, label='P Term (Proportional)', 
                    color='#FF8C00', linewidth=2.5, alpha=0.9)
    # I项（积分项）
    ax_pid_all.plot(df['Time_sec'], i_term, label='I Term (Integral)', 
                    color='#00CED1', linewidth=2.5, alpha=0.9)
    # D项（微分项）
    if has_d_term:
        ax_pid_all.plot(df['Time_sec'], d_term, label='D Term (Derivative)', 
                        color='#FF69B4', linewidth=2.5, alpha=0.9)
    # 总PID输出
    ax_pid_all.plot(df['Time_sec'], pi_output, label='Total PID Output', 
                    color='#DC143C', linewidth=3, linestyle='--')
    # 前馈量
    if 'Feedforward(mA)' in df.columns:
        ax_pid_all.plot(df['Time_sec'], df['Feedforward(mA)'], label='Feedforward', 
                        color='#32CD32', linewidth=2.5, linestyle='-.')
    
    ax_pid_all.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    # 标记关键时间点
    if weight_add_time is not None:
        ax_pid_all.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, 
                          label=f'Weight Added ({weight_add_time:.2f}s)')
    if stable_time is not None:
        ax_pid_all.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, 
                          label=f'Stable ({stable_time:.2f}s)')

    ax_pid_all.set_ylabel('Current (mA)', fontsize=12)
    ax_pid_all.set_title('Incremental PID Components (P, I, D) + Total PID Output + Feedforward', 
                        fontsize=13, fontweight='bold')
    ax_pid_all.legend(loc='upper right', fontsize=11, ncol=2)
    ax_pid_all.grid(True, alpha=0.3)
    ax_pid_all.tick_params(labelsize=10)

    # 子图2: 力（压力）和电流的关系（双Y轴）
    ax_force_current = axes[1]
    
    # 左Y轴：压力（力）
    ax_force_current.plot(df['Time_sec'], pressure, label='Pressure (Force)', 
                         color='blue', linewidth=2.5)
    ax_force_current.axhline(y=f0, color='green', linestyle='--', linewidth=1.5, 
                            label=f'Target F0 = {f0:.2f} kg')
    ax_force_current.set_ylabel('Pressure (kg)', color='blue', fontsize=12)
    ax_force_current.tick_params(axis='y', labelcolor='blue')
    
    # 右Y轴：实际电流
    ax_force_current_twin = ax_force_current.twinx()
    ax_force_current_twin.plot(df['Time_sec'], current, label='Actual Current', 
                               color='red', linewidth=2.5, linestyle='--')
    ax_force_current_twin.plot(df['Time_sec'], target_current, label='Target Current', 
                               color='orange', linewidth=2, linestyle=':')
    ax_force_current_twin.set_ylabel('Current (mA)', color='red', fontsize=12)
    ax_force_current_twin.tick_params(axis='y', labelcolor='red')

    # 标记关键时间点
    if weight_add_time is not None:
        ax_force_current.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax_force_current.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)

    ax_force_current.set_xlabel('Time (s)', fontsize=12)
    ax_force_current.set_title('Force (Pressure) vs Current Relationship', fontsize=13, fontweight='bold')
    
    # 合并两个Y轴的图例
    lines1, labels1 = ax_force_current.get_legend_handles_labels()
    lines2, labels2 = ax_force_current_twin.get_legend_handles_labels()
    ax_force_current.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=11)
    
    ax_force_current.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为总标题留出空间
    pi_output_file = os.path.join(os.getcwd(), base_name.replace('_performance_analysis.png', '_PID_detailed.png'))
    plt.savefig(pi_output_file, dpi=150, bbox_inches='tight')
    print(f"PID详细图表已保存: {pi_output_file}")
    plt.close()

    # 生成总结
    print("\n" + "=" * 80)
    print("【算法性能总结】")
    print("=" * 80)
    
    # 准备关键时间点字符串
    weight_add_str = f"{weight_add_time:.2f}" if weight_add_time is not None else "N/A"
    stable_time_str = f"{stable_time:.2f}" if stable_time is not None else "N/A"
    settling_time_str = f"{settling_time:.2f}" if settling_time is not None else "N/A"
    data_source_str = 'Real Data' if using_real_data else 'Calculated'
    
    print(f"""
Algorithm Performance Summary
{'='*60}

Pressure Control:
  Target: {f0:.2f} kg
  Range: {pressure.min():.2f} ~ {pressure.max():.2f} kg
  Steady Error: {steady_error.mean():.3f} +/- {steady_error.std():.3f} kg
  Overshoot: {pressure_overshoot:.2f}%

Current Control:
  Range: {current.min():.1f} ~ {current.max():.1f} mA
  Tracking Error: {current_error.mean():.2f} mA (avg)

Motor Response:
  Max Speed: {motor_speed.max():.1f} rpm
  Avg Speed: {motor_speed.mean():.1f} rpm

Position:
  Total Displacement: {position.max() - position.min():.2f} mm

Key Timing:
  Weight Added: {weight_add_str} s
  Stable (dF=0): {stable_time_str} s
  Settling Time: {settling_time_str} s

PI Control:
  P Term: {p_term.min():.1f} ~ {p_term.max():.1f} mA
  I Term: {i_term.min():.1f} ~ {i_term.max():.1f} mA
  PI Output: {pi_output.min():.1f} ~ {pi_output.max():.1f} mA
  Data Source: {data_source_str}

Tuning Suggestions:
  Slow response: Increase Kp
  Static error: Increase Ki
  Oscillation: Decrease Kp or increase deadzone
  Large overshoot: Decrease Kp or increase Ki
{'='*60}
    """)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_gravity_performance.py <csv_file> [sample_period_ms]")
        print("  sample_period_ms: 采样周期（毫秒），默认10ms（100Hz）")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    sample_period_ms = 10  # 默认10ms = 100Hz
    
    if len(sys.argv) >= 3:
        sample_period_ms = int(sys.argv[2])
        print(f"使用指定采样周期: {sample_period_ms}ms ({int(1000/sample_period_ms)}Hz)")
    
    analyze_gravity_performance(csv_file, sample_period_ms)
