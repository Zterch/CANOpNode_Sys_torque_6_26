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

def analyze_gravity_performance(csv_file):
    # 读取CSV数据
    df = pd.read_csv(csv_file, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    
    # 创建时间序列（秒）
    df['Time_sec'] = np.arange(len(df)) * 0.02  # 50Hz采样
    
    print("=" * 80)
    print("重力卸载算法性能分析报告")
    print("=" * 80)
    
    # 1. 基本统计信息
    print("\n【1. 基本统计信息】")
    print(f"数据点数: {len(df)}")
    print(f"时间跨度: {df['Time_sec'].iloc[-1]:.1f} 秒")
    print(f"采样频率: 50 Hz")
    
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
    
    # 计算稳态误差
    steady_state_start = int(len(df) * 0.3)  # 后70%作为稳态
    steady_pressure = pressure.iloc[steady_state_start:]
    steady_error = steady_pressure - f0
    print(f"\n稳态压力偏差:")
    print(f"  平均值: {steady_error.mean():.3f} kg")
    print(f"  标准差: {steady_error.std():.3f} kg")
    print(f"  最大偏差: {steady_error.abs().max():.3f} kg")
    
    # 从CSV读取真实的PI控制项数据
    # 如果CSV中有PI_P(mA)和PI_I(mA)列，使用真实数据
    if 'PI_P(mA)' in df.columns and 'PI_I(mA)' in df.columns:
        # 使用采集的真实数据
        p_term = df['PI_P(mA)']
        i_term = df['PI_I(mA)']
        pi_output = p_term + i_term
        using_real_data = True
        
        print(f"\n【PI控制分析 - 使用真实采集数据】")
        print(f"P项范围: {p_term.min():.2f} ~ {p_term.max():.2f} mA")
        print(f"I项范围: {i_term.min():.2f} ~ {i_term.max():.2f} mA")
        print(f"PI输出范围: {pi_output.min():.2f} ~ {pi_output.max():.2f} mA")
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
        DT = 0.02   # ALGO_CONTROL_PERIOD_S = 20ms (固定值)
        
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
    
    # 图8b: 力传感器（压力）和实际电流的关系（新增）
    ax8b = fig.add_subplot(gs[5, 1])
    ax8b.plot(df['Time_sec'], pressure, label='Pressure (Force)', color='blue', linewidth=2)
    ax8b_twin = ax8b.twinx()
    ax8b_twin.plot(df['Time_sec'], current, label='Actual Current', color='red', linewidth=2, linestyle='--')
    ax8b.set_xlabel('Time (s)')
    ax8b.set_ylabel('Pressure (kg)', color='blue')
    ax8b_twin.set_ylabel('Current (mA)', color='red')
    ax8b.set_title('Pressure vs Current Relationship')
    ax8b.legend(loc='upper left')
    ax8b_twin.legend(loc='upper right')
    ax8b.grid(True, alpha=0.3)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax8b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax8b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    
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
    
    # 创建第二个图表 - PI控制项放大图
    fig2, axes = plt.subplots(3, 1, figsize=(16, 12))
    fig2.suptitle('PI Control Components - Detailed View', fontsize=16, fontweight='bold')
    
    # 子图1: P项和I项对比
    ax_pi_1 = axes[0]
    ax_pi_1.plot(df['Time_sec'], p_term, label='P Term (Proportional)', color='orange', linewidth=2)
    ax_pi_1.plot(df['Time_sec'], i_term, label='I Term (Integral)', color='cyan', linewidth=2)
    ax_pi_1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax_pi_1.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, label=f'Weight Added ({weight_add_time:.2f}s)')
    if stable_time is not None:
        ax_pi_1.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, label=f'Stable ({stable_time:.2f}s)')
    
    ax_pi_1.set_ylabel('Current (mA)', fontsize=12)
    ax_pi_1.set_title('P Term vs I Term Comparison', fontsize=13, fontweight='bold')
    ax_pi_1.legend(loc='upper right', fontsize=11)
    ax_pi_1.grid(True, alpha=0.3)
    ax_pi_1.tick_params(labelsize=10)
    
    # 子图2: P项单独显示
    ax_pi_2 = axes[1]
    ax_pi_2.plot(df['Time_sec'], p_term, label='P Term', color='orange', linewidth=2.5)
    ax_pi_2.fill_between(df['Time_sec'], p_term, alpha=0.3, color='orange')
    ax_pi_2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax_pi_2.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax_pi_2.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax_pi_2.set_ylabel('Current (mA)', fontsize=12)
    ax_pi_2.set_title(f'P Term (Proportional) - Kp × DeltaF', fontsize=13, fontweight='bold')
    ax_pi_2.legend(loc='upper right', fontsize=11)
    ax_pi_2.grid(True, alpha=0.3)
    ax_pi_2.tick_params(labelsize=10)
    
    # 子图3: I项单独显示
    ax_pi_3 = axes[2]
    ax_pi_3.plot(df['Time_sec'], i_term, label='I Term', color='cyan', linewidth=2.5)
    ax_pi_3.fill_between(df['Time_sec'], i_term, alpha=0.3, color='cyan')
    ax_pi_3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax_pi_3.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax_pi_3.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    
    ax_pi_3.set_ylabel('Current (mA)', fontsize=12)
    ax_pi_3.set_xlabel('Time (s)', fontsize=12)
    ax_pi_3.set_title(f'I Term (Integral) - Ki × ∫DeltaF dt', fontsize=13, fontweight='bold')
    ax_pi_3.legend(loc='upper right', fontsize=11)
    ax_pi_3.grid(True, alpha=0.3)
    ax_pi_3.tick_params(labelsize=10)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # 为总标题留出空间
    pi_output_file = os.path.join(os.getcwd(), base_name.replace('_performance_analysis.png', '_PI_detailed.png'))
    plt.savefig(pi_output_file, dpi=150, bbox_inches='tight')
    print(f"PI详细图表已保存: {pi_output_file}")
    plt.close()
    
    # 创建第三个图表 - 力与电流关系图
    fig3, axes3 = plt.subplots(2, 2, figsize=(16, 12))
    fig3.suptitle('Force vs Current Relationship Analysis', fontsize=16, fontweight='bold')
    
    # 子图1: DeltaF vs 实际电流（散点图）
    ax_fc_1 = axes3[0, 0]
    scatter = ax_fc_1.scatter(delta_f, current, c=df['Time_sec'], cmap='viridis', alpha=0.6, s=20)
    ax_fc_1.set_xlabel('DeltaF (kg)', fontsize=12)
    ax_fc_1.set_ylabel('Actual Current (mA)', fontsize=12)
    ax_fc_1.set_title('DeltaF vs Actual Current (colored by time)')
    ax_fc_1.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax_fc_1, label='Time (s)')
    
    # 添加理论前馈线
    delta_f_range = np.linspace(delta_f.min(), delta_f.max(), 100)
    theoretical_feedforward = 50.0 + delta_f_range * 273.0
    ax_fc_1.plot(delta_f_range, theoretical_feedforward, 'r--', linewidth=2, label='Theoretical Feedforward (50+273*dF)')
    ax_fc_1.legend()
    
    # 子图2: DeltaF vs 目标电流（散点图）
    ax_fc_2 = axes3[0, 1]
    ax_fc_2.scatter(delta_f, target_current, c='orange', alpha=0.6, s=20, label='Target Current')
    if 'Feedforward(mA)' in df.columns:
        ax_fc_2.scatter(delta_f, df['Feedforward(mA)'], c='red', alpha=0.6, s=20, label='Feedforward Current')
    ax_fc_2.set_xlabel('DeltaF (kg)', fontsize=12)
    ax_fc_2.set_ylabel('Current (mA)', fontsize=12)
    ax_fc_2.set_title('DeltaF vs Target/Feedforward Current')
    ax_fc_2.grid(True, alpha=0.3)
    ax_fc_2.legend()
    
    # 子图3: 电流-力关系的时间序列
    ax_fc_3 = axes3[1, 0]
    ax_fc_3.plot(df['Time_sec'], delta_f, label='DeltaF', color='purple', linewidth=2)
    ax_fc_3_twin = ax_fc_3.twinx()
    ax_fc_3_twin.plot(df['Time_sec'], current, label='Actual Current', color='blue', linewidth=2, alpha=0.7)
    ax_fc_3_twin.plot(df['Time_sec'], target_current, label='Target Current', color='orange', linewidth=2, linestyle='--', alpha=0.7)
    ax_fc_3.set_xlabel('Time (s)', fontsize=12)
    ax_fc_3.set_ylabel('DeltaF (kg)', color='purple', fontsize=12)
    ax_fc_3_twin.set_ylabel('Current (mA)', color='blue', fontsize=12)
    ax_fc_3.set_title('DeltaF and Current vs Time')
    ax_fc_3.grid(True, alpha=0.3)
    ax_fc_3.legend(loc='upper left')
    ax_fc_3_twin.legend(loc='upper right')
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax_fc_3.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax_fc_3.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    
    # 子图4: 电流跟踪误差分析
    ax_fc_4 = axes3[1, 1]
    current_error = target_current - current
    ax_fc_4.plot(df['Time_sec'], current_error, label='Current Tracking Error', color='red', linewidth=1.5)
    ax_fc_4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax_fc_4.axhline(y=current_error.mean(), color='blue', linestyle='--', linewidth=1, label=f'Mean Error: {current_error.mean():.1f}mA')
    ax_fc_4.fill_between(df['Time_sec'], current_error, alpha=0.3, color='red')
    ax_fc_4.set_xlabel('Time (s)', fontsize=12)
    ax_fc_4.set_ylabel('Current Error (mA)', fontsize=12)
    ax_fc_4.set_title('Current Tracking Error (Target - Actual)')
    ax_fc_4.grid(True, alpha=0.3)
    ax_fc_4.legend()
    
    # 标记关键时间点
    if weight_add_time is not None:
        ax_fc_4.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax_fc_4.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    force_current_file = os.path.join(os.getcwd(), base_name.replace('_performance_analysis.png', '_force_current.png'))
    plt.savefig(force_current_file, dpi=150, bbox_inches='tight')
    print(f"力-电流关系图表已保存: {force_current_file}")
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
        print("Usage: python3 analyze_gravity_performance.py <csv_file>")
        sys.exit(1)
    
    analyze_gravity_performance(sys.argv[1])
