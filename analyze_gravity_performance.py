#!/usr/bin/env python3
"""
重力卸载算法性能分析脚本
展示算法效果和恒力控制性能
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import re
from matplotlib.gridspec import GridSpec

# ==================== 全局配置参数 ====================
DELTAF_THRESHOLD_KG_1 = 0.15
DELTAF_THRESHOLD_KG_2 = 0.3
# ====================================================

def analyze_gravity_performance(csv_file, sample_period_ms=None):
    # 读取CSV数据
    df = pd.read_csv(csv_file, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    
    # ------- 解析真实时间戳，自动检测采样周期 -------
    using_real_time = False
    if 'Time' in df.columns:
        try:
            # 尝试解析时间格式，支持多种分隔符
            df['Datetime'] = pd.to_datetime(df['Time'], format='%H:%M:%S.%f')
            start_time = df['Datetime'].iloc[0]
            df['Time_real'] = (df['Datetime'] - start_time).dt.total_seconds()
            using_real_time = True
            
            # 自动检测平均采样周期
            if sample_period_ms is None:
                avg_dt_ms = (df['Time_real'].diff().median() * 1000)
                # 限制到合理的范围（5ms或10ms）
                if avg_dt_ms < 7.5:
                    sample_period_ms = 5
                else:
                    sample_period_ms = 10
                print(f"[INFO] 自动检测到采样周期: {sample_period_ms} ms ({1000/sample_period_ms:.0f} Hz)")
        except:
            # 如果解析失败，使用固定周期
            df['Time_real'] = df['Time_sec'].copy() if 'Time_sec' in df.columns else np.arange(len(df)) * 0.01
            using_real_time = False
    
    if sample_period_ms is None:
        sample_period_ms = 10
    
    sample_period_s = sample_period_ms / 1000.0
    # 保留固定周期时间序列作为备用
    df['Time_sec'] = np.arange(len(df)) * sample_period_s
    
    if not using_real_time:
        df['Time_real'] = df['Time_sec'].copy()
    
    print("=" * 80)
    print("重力卸载算法性能分析报告")
    print("=" * 80)
    
    # 1. 基本统计
    print("\n【1. 基本统计信息】")
    print(f"数据点数: {len(df)}")
    print(f"时间跨度 (固定周期): {df['Time_sec'].iloc[-1]:.1f} 秒")
    if using_real_time:
        print(f"时间跨度 (实际): {df['Time_real'].iloc[-1] - df['Time_real'].iloc[0]:.3f} 秒")
    print(f"采样频率: {int(1/sample_period_s)} Hz")
    print(f"采样周期: {sample_period_ms} ms")
    
    # 2. 压力控制性能
    print("\n【2. 压力控制性能分析】")
    if 'PressureRaw(kg)' in df.columns and 'PressureFiltered(kg)' in df.columns:
        pressure_raw = df['PressureRaw(kg)']
        pressure_filtered = df['PressureFiltered(kg)']
        pressure = pressure_filtered
        has_raw_filtered = True
        print("  使用新的CSV格式（包含原始和滤波后压力值）")
    else:
        pressure = df['Pressure(kg)']
        pressure_raw = None
        pressure_filtered = None
        has_raw_filtered = False
        print("  使用旧的CSV格式（仅压力值）")
    
    # 重量数据
    if 'WeightRaw(kg)' in df.columns and 'WeightFiltered(kg)' in df.columns:
        weight_raw = df['WeightRaw(kg)']
        weight_filtered = df['WeightFiltered(kg)']
        has_weight_data = True
        print("  检测到新的重量采集数据（UART/TTL模块，100Hz）")
    else:
        weight_raw = None
        weight_filtered = None
        has_weight_data = False
    
    f0 = df['F0(kg)'].iloc[0]
    delta_f = df['DeltaF']
    print(f"目标压力 (F0): {f0:.3f} kg")
    
    if has_raw_filtered:
        print(f"\n原始压力值: min={pressure_raw.min():.3f}, max={pressure_raw.max():.3f}, mean={pressure_raw.mean():.3f}, std={pressure_raw.std():.3f}")
        print(f"滤波后压力值: min={pressure_filtered.min():.3f}, max={pressure_filtered.max():.3f}, mean={pressure_filtered.mean():.3f}, std={pressure_filtered.std():.3f}")
        print(f"滤波效果: 标准差降低 {(1 - pressure_filtered.std()/pressure_raw.std())*100:.1f}%")
    else:
        print(f"实际压力 - min={pressure.min():.3f}, max={pressure.max():.3f}, mean={pressure.mean():.3f}, std={pressure.std():.3f}")
    
    if has_weight_data:
        print(f"\n【2.1 重量采集模块数据】")
        print(f"原始重量: min={weight_raw.min():.3f}, max={weight_raw.max():.3f}, mean={weight_raw.mean():.3f}, std={weight_raw.std():.3f}")
        print(f"滤波重量: min={weight_filtered.min():.3f}, max={weight_filtered.max():.3f}, mean={weight_filtered.mean():.3f}, std={weight_filtered.std():.3f}")
    
    # 稳态误差
    steady_start = int(len(df) * 0.3)
    steady_pressure = pressure.iloc[steady_start:]
    steady_error = steady_pressure - f0
    print(f"\n稳态压力偏差: 平均 {steady_error.mean():.3f} kg, 标准差 {steady_error.std():.3f} kg, 最大偏差 {steady_error.abs().max():.3f} kg")
    
    # ---------- 控制算法输出处理（ADRC优先，兼容旧PID） ----------
    has_d_term = False
    has_last_current = False
    d_term = None
    pi_last_current = None
    using_real_data = False
    using_adrc_data = False
    adrc_u0 = None
    adrc_z1 = None
    adrc_z2 = None
    adrc_output = None
    adrc_kp = None
    adrc_p_gain = None
    
    # 优先检测新 ADRC 列
    if 'ADRC_U0(Nm)' in df.columns and 'ADRC_Z1(kg)' in df.columns:
        adrc_u0 = df['ADRC_U0(Nm)']
        adrc_z1 = df['ADRC_Z1(kg)']
        adrc_z2 = df['ADRC_Z2(Nm)']
        adrc_output = df['ADRC_LastTorque(Nm)']
        adrc_kp = df['ADRC_KP'] if 'ADRC_KP' in df.columns else None
        adrc_p_gain = df['ADRC_PGain'] if 'ADRC_PGain' in df.columns else None
        using_adrc_data = True
        using_real_data = True
        pi_output = adrc_output
        p_term = adrc_u0
        i_term = adrc_z2
        d_term = None
        has_d_term = False
        print(f"\n【ADRC控制分析 - 使用真实采集数据】")
        if adrc_kp is not None:
            print(f"  ADRC KP: {adrc_kp.iloc[0]:.2f}")
        if adrc_p_gain is not None:
            print(f"  ADRC 动态P增益范围: {adrc_p_gain.min():.2f} ~ {adrc_p_gain.max():.2f}")
        print(f"  u0 范围: {adrc_u0.min():.3f} ~ {adrc_u0.max():.3f} Nm")
        print(f"  z1 (估计DeltaF) 范围: {adrc_z1.min():.3f} ~ {adrc_z1.max():.3f} kg")
        print(f"  z2 (估计扰动) 范围: {adrc_z2.min():.3f} ~ {adrc_z2.max():.3f} Nm")
        print(f"  ADRC输出扭矩范围: {adrc_output.min():.3f} ~ {adrc_output.max():.3f} Nm")
    elif 'PI_P(Nm)' in df.columns and 'PI_I(Nm)' in df.columns:
        # 旧PID数据分支（兼容）
        p_term = df['PI_P(Nm)']
        i_term = df['PI_I(Nm)']
        if 'PI_D(Nm)' in df.columns:
            d_term = df['PI_D(Nm)']
            has_d_term = True
            pi_output = p_term + i_term + d_term
        else:
            d_term = None
            has_d_term = False
            pi_output = p_term + i_term
        using_real_data = True

        if 'PI_LastTorque(Nm)' in df.columns:
            pi_last_current = df['PI_LastTorque(Nm)']
            has_last_current = True
        else:
            pi_last_current = None
            has_last_current = False

        print(f"\n【PID控制分析 - 使用真实采集数据（旧格式）】")
        print(f"P项范围: {p_term.min():.2f} ~ {p_term.max():.2f} Nm")
        print(f"I项范围: {i_term.min():.2f} ~ {i_term.max():.2f} Nm")
        if has_d_term:
            print(f"D项范围: {d_term.min():.2f} ~ {d_term.max():.2f} Nm")
        print(f"PID输出范围: {pi_output.min():.2f} ~ {pi_output.max():.2f} Nm")
        if has_last_current:
            print(f"PID累积扭矩(LastTorque)范围: {pi_last_current.min():.2f} ~ {pi_last_current.max():.2f} Nm")
    else:
        # 计算分支（若无法读取配置则使用默认值）
        config_file = '/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys/algorithms/algorithm_config.h'
        
        def read_config_value(filename, pattern):
            try:
                with open(filename, 'r') as f:
                    content = f.read()
                    match = re.search(pattern, content)
                    if match:
                        return float(match.group(1))
            except Exception as e:
                print(f"Warning: Could not read config: {e}")
            return None
        
        KP = read_config_value(config_file, r'CLUTCH_PI_KP\s+(\d+\.?\d*)f?')
        KI = read_config_value(config_file, r'CLUTCH_PI_KI\s+(\d+\.?\d*)f?')
        DT = sample_period_s
        
        if KP is None:
            KP = 0.5
            print(f"Warning: Could not read KP from config, using default: {KP}")
        if KI is None:
            KI = 1000.0
            print(f"Warning: Could not read KI from config, using default: {KI}")
        
        p_term = KP * delta_f
        i_term = np.zeros(len(df))
        integral = 0.0
        for i in range(len(df)):
            integral += KI * delta_f.iloc[i] * DT
            integral = np.clip(integral, -100.0, 100.0)
            i_term[i] = integral
        pi_output = p_term + i_term
        using_real_data = False
        
        print(f"\n【PI控制分析 - 使用计算数据（CSV中没有PI列）】")
        print(f"Kp = {KP}, Ki = {KI}")
        print(f"P项范围: {p_term.min():.2f} ~ {p_term.max():.2f} mA")
        print(f"I项范围: {i_term.min():.2f} ~ {i_term.max():.2f} mA")
        print(f"PI输出范围: {pi_output.min():.2f} ~ {pi_output.max():.2f} mA")
    
    # 电流控制
    print("\n【3. 电流控制性能分析】")
    current = df['Current(A)'] * 1000
    target_current = df['TargetCurrent(A)'] * 1000
    print(f"实际电流 - min={current.min():.1f} mA, max={current.max():.1f} mA, mean={current.mean():.1f} mA, std={current.std():.1f} mA")
    print(f"目标电流 - min={target_current.min():.1f} mA, max={target_current.max():.1f} mA")
    current_error = (current - target_current).abs()
    print(f"电流跟踪误差: 平均 {current_error.mean():.2f} mA, 最大 {current_error.max():.2f} mA")
    
    # 电机速度
    print("\n【4. 电机速度分析】")
    motor_speed = df['MotorSpeed(rpm)']
    print(f"电机速度 - min={motor_speed.min():.1f} rpm, max={motor_speed.max():.1f} rpm, mean={motor_speed.mean():.1f} rpm, std={motor_speed.std():.1f} rpm")
    
    # 位置变化
    print("\n【5. 位置变化分析】")
    position = df['RopeLength(m)'] * 1000
    print(f"绳子长度变化: {position.min():.2f} mm -> {position.max():.2f} mm, 总位移: {position.max() - position.min():.2f} mm")
    
    # 关键时间点检测（使用真实时间或固定时间）
    time_axis = df['Time_real'] if using_real_time else df['Time_sec']
    pressure_diff = pressure.diff()
    sig_idx = np.where(pressure_diff > 0.05)[0]
    weight_add_time = None
    stable_time = None
    settling_time = None
    if len(sig_idx) > 0:
        weight_add_time = time_axis.iloc[sig_idx[0]]
        print(f"重量添加时间点: {weight_add_time:.2f} 秒 (压力突增)")
        for i in range(sig_idx[0] + 10, len(df)):
            if abs(delta_f.iloc[i]) < 0.02:
                stable_time = time_axis.iloc[i]
                settling_time = stable_time - weight_add_time
                print(f"控制稳定时间点: {stable_time:.2f} 秒 (DeltaF首次回零)")
                print(f"调节时间: {settling_time:.2f} 秒")
                break
        if stable_time is None:
            print("未检测到稳定点（DeltaF未回零）")
    else:
        print("未检测到重量添加事件")
    
    # 恒力控制评估
    print("\n【7. 恒力控制效果评估】")
    pressure_overshoot = (pressure.max() - f0) / f0 * 100 if f0 > 0 else 0
    print(f"压力超调量: {pressure_overshoot:.2f}%")
    tolerance = 0.5
    within_tolerance = np.abs(pressure - f0) < tolerance
    if np.any(within_tolerance):
        settling_idx = np.where(within_tolerance)[0][0]
        settling_time2 = time_axis.iloc[settling_idx]
        print(f"调节时间 (±{tolerance}kg): {settling_time2:.2f} 秒")
    
    # ========== AlgoDeltaF 绝对值统计（使用真实时间戳精确统计） ==========
    print("\n【8. AlgoDeltaF 绝对值统计分析（精确时间）】")
    algo_delta_f = df['AlgoDeltaF(kg)'] if 'AlgoDeltaF(kg)' in df.columns else df['DeltaF']
    abs_algo_df = algo_delta_f.abs()
    
    # 使用真实时间计算间隔
    time_real = df['Time_real'].values
    time_diffs = np.zeros(len(df))
    time_diffs[1:] = np.diff(time_real)  # 每个点与前一点的实际间隔
    
    THRESHOLD_1 = 0.2
    THRESHOLD_2 = 0.3
    
    over_02_mask = abs_algo_df > THRESHOLD_1
    over_03_mask = abs_algo_df > THRESHOLD_2
    
    # 累加超过阈值的间隔
    duration_over_02 = time_diffs[over_02_mask].sum()
    duration_over_03 = time_diffs[over_03_mask].sum()
    
    total_duration = time_real[-1] - time_real[0]
    pct_over_02 = (duration_over_02 / total_duration) * 100 if total_duration > 0 else 0
    pct_over_03 = (duration_over_03 / total_duration) * 100 if total_duration > 0 else 0
    
    max_abs_df = abs_algo_df.max()
    max_abs_df_time = time_real[abs_algo_df.idxmax()]
    
    print(f"  总持续时间: {total_duration:.3f} 秒")
    print(f"  超过 ±{THRESHOLD_1}kg 的采样点数: {over_02_mask.sum()} / {len(df)} ({over_02_mask.sum()/len(df)*100:.1f}%)")
    print(f"  超过 ±{THRESHOLD_1}kg 的总持续时间: {duration_over_02:.3f} 秒 (占比 {pct_over_02:.1f}%)")
    print(f"  超过 ±{THRESHOLD_2}kg 的采样点数: {over_03_mask.sum()} / {len(df)} ({over_03_mask.sum()/len(df)*100:.1f}%)")
    print(f"  超过 ±{THRESHOLD_2}kg 的总持续时间: {duration_over_03:.3f} 秒 (占比 {pct_over_03:.1f}%)")
    print(f"  AlgoDeltaF 最大绝对值: {max_abs_df:.3f} kg (发生在 {max_abs_df_time:.3f} 秒)")
    
    # 重量添加后的时间统计
    if weight_add_time is not None:
        # 找到重量添加时刻的索引
        add_idx = np.argmin(np.abs(time_real - weight_add_time))
        # 对重量添加后的数据进行统计
        over_02_after = over_02_mask.iloc[add_idx:]
        over_03_after = over_03_mask.iloc[add_idx:]
        duration_02_after = time_diffs[add_idx:][over_02_after].sum()
        duration_03_after = time_diffs[add_idx:][over_03_after].sum()
        total_after = time_real[-1] - time_real[add_idx]
        pct_02_after = (duration_02_after / total_after) * 100 if total_after > 0 else 0
        pct_03_after = (duration_03_after / total_after) * 100 if total_after > 0 else 0
        print(f"\n  重量添加后 (>={weight_add_time:.2f}s):")
        print(f"    超过 ±{THRESHOLD_1}kg 持续时间: {duration_02_after:.3f} 秒 (占比 {pct_02_after:.1f}%)")
        print(f"    超过 ±{THRESHOLD_2}kg 持续时间: {duration_03_after:.3f} 秒 (占比 {pct_03_after:.1f}%)")
    
    # 稳态阶段（后70%数据）统计
    steady_start_idx = int(len(df) * 0.3)
    steady_abs_df = abs_algo_df.iloc[steady_start_idx:]
    steady_mean_df = steady_abs_df.mean()
    steady_std_df = steady_abs_df.std()
    steady_max_df = steady_abs_df.max()
    print(f"\n  稳态阶段 (后70%数据):")
    print(f"    AlgoDeltaF 平均绝对值: {steady_mean_df:.3f} kg")
    print(f"    AlgoDeltaF 标准差: {steady_std_df:.3f} kg")
    print(f"    AlgoDeltaF 最大绝对值: {steady_max_df:.3f} kg")
    
    # ==================== 绘图（使用固定时间轴） ====================
    print("\n正在生成分析图表...")
    # 使用固定周期时间绘制，保证图像一致性
    # 绘图时间轴：优先使用真实时间戳
    plot_time = df['Time_real']
    
    fig = plt.figure(figsize=(18, 26))
    gs = GridSpec(13, 2, figure=fig, hspace=0.35, wspace=0.3)
    
    # ---------- 图1: 压力控制效果 ----------
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(plot_time, pressure, label='Filtered Pressure', color='blue', linewidth=2)
    ax1.axhline(y=f0, color='r', linestyle='--', linewidth=2, label=f'Target F0={f0:.2f}kg')
    ax1.fill_between(plot_time, f0 - 0.5, f0 + 0.5, alpha=0.2, color='green', label='±0.5kg')
    if weight_add_time is not None:
        ax1.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, label=f'Weight Added {weight_add_time:.2f}s')
    if stable_time is not None:
        ax1.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, label=f'Stable {stable_time:.2f}s')
    ax1.set_ylabel('Pressure (kg)')
    ax1.set_title('Pressure Control Performance', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([f0 - 1, pressure.max() + 0.5])
    
    # ---------- 图1b: 原始vs滤波 ----------
    if has_raw_filtered:
        ax1b = fig.add_subplot(gs[1, :])
        ax1b.plot(plot_time, pressure_raw, label='Raw Pressure', color='red', linewidth=1.5, linestyle='--', alpha=0.7)
        ax1b.plot(plot_time, pressure_filtered, label='Filtered Pressure', color='blue', linewidth=2)
        ax1b.axhline(y=f0, color='green', linestyle=':', linewidth=2)
        if weight_add_time is not None:
            ax1b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
        if stable_time is not None:
            ax1b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
        ax1b.set_ylabel('Pressure (kg)')
        ax1b.set_title('Raw vs Filtered Pressure')
        ax1b.legend()
        ax1b.grid(True, alpha=0.3)
    
    # ---------- 图2: DeltaF + 目标扭矩 + AlgoDeltaF统计标注 ----------
    ax2 = fig.add_subplot(gs[2, 0])
    ax2.plot(plot_time, delta_f, label='DeltaF (Data)', color='purple', linewidth=1.5)
    if 'AlgoDeltaF(kg)' in df.columns:
        ax2.plot(plot_time, df['AlgoDeltaF(kg)'], label='AlgoDeltaF', color='red', linewidth=1.5, linestyle='--', alpha=0.7)
    ax2.axhline(y=0, color='k', linewidth=0.5)
    
    # 添加阈值线和统计标注
    ax2.axhline(y=0.2, color='blue', linestyle=':', linewidth=1.5, alpha=0.7, label='±0.2kg')
    ax2.axhline(y=-0.2, color='blue', linestyle=':', linewidth=1.5, alpha=0.7)
    ax2.axhline(y=0.3, color='darkred', linestyle=':', linewidth=1.5, alpha=0.7, label='±0.3kg')
    ax2.axhline(y=-0.3, color='darkred', linestyle=':', linewidth=1.5, alpha=0.7)
    
    # 在图中标注统计信息
    stats_text = f"|AlgoDeltaF| > 0.2kg: {duration_over_02:.3f}s ({pct_over_02:.1f}%)\n"
    stats_text += f"|AlgoDeltaF| > 0.3kg: {duration_over_03:.3f}s ({pct_over_03:.1f}%)\n"
    stats_text += f"Max |AlgoDeltaF|: {max_abs_df:.3f}kg @ {max_abs_df_time:.3f}s"
    ax2.text(0.98, 0.02, stats_text, transform=ax2.transAxes, fontsize=8,
             verticalalignment='bottom', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax2.set_ylabel('DeltaF (kg)', color='purple')
    ax2.tick_params(axis='y', labelcolor='purple')
    if 'TargetTorque(Nm)' in df.columns:
        ax2_twin = ax2.twinx()
        ax2_twin.plot(plot_time, df['TargetTorque(Nm)'], label='Target Torque', color='orange', linewidth=2, linestyle='-.')
        ax2_twin.set_ylabel('Target Torque (Nm)', color='orange')
        ax2_twin.tick_params(axis='y', labelcolor='orange')
        ax2_twin.legend(loc='upper right')
    if weight_add_time is not None:
        ax2.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax2.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    ax2.set_title('DeltaF and Target Torque (with AlgoDeltaF Stats)')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)
    
    # ---------- 图2b: DeltaF与ADRC/PID控制项 ----------
    ax2b = fig.add_subplot(gs[3, :])
    ax2b.plot(plot_time, delta_f, label='DeltaF', color='purple', linewidth=2)
    ax2b.axhline(y=DELTAF_THRESHOLD_KG_1, color='red', linestyle='--', linewidth=2, label=f'Thr1 ±{DELTAF_THRESHOLD_KG_1}kg')
    ax2b.axhline(y=-DELTAF_THRESHOLD_KG_1, color='red', linestyle='--', linewidth=2)
    ax2b.axhline(y=DELTAF_THRESHOLD_KG_2, color='orange', linestyle='-.', linewidth=2, label=f'Thr2 ±{DELTAF_THRESHOLD_KG_2}kg')
    ax2b.axhline(y=-DELTAF_THRESHOLD_KG_2, color='orange', linestyle='-.', linewidth=2)
    ax2b.fill_between(plot_time, -DELTAF_THRESHOLD_KG_1, DELTAF_THRESHOLD_KG_1, alpha=0.1, color='yellow')
    ax2b.set_ylabel('DeltaF (kg)', color='purple')
    ax2b.tick_params(axis='y', labelcolor='purple')

    ax2b_twin = ax2b.twinx()
    if using_adrc_data:
        # ADRC数据：绘制u0, z1, z2, 最终输出
        ax2b_twin.plot(plot_time, adrc_u0, label='ADRC u0', color='red', linewidth=1.5, linestyle='--', alpha=0.8)
        ax2b_twin.plot(plot_time, adrc_z2, label='ADRC z2', color='cyan', linewidth=1.5, linestyle=':', alpha=0.8)
        ax2b_twin.plot(plot_time, adrc_output, label='ADRC Output Torque', color='green', linewidth=2)
        if adrc_p_gain is not None:
            ax2b_twin.plot(plot_time, adrc_p_gain, label='ADRC P Gain Mult', color='magenta', linewidth=1.5, linestyle='-.')
        ax2b_twin.set_ylabel('ADRC Value (Nm)', color='green')
    elif 'PI_P(Nm)' in df.columns:
        if 'PI_P_Raw(Nm)' in df.columns:
            ax2b_twin.plot(plot_time, df['PI_P_Raw(Nm)'], label='P Raw', color='red', linewidth=1.5, linestyle='--', alpha=0.8)
        if 'PI_P_Filtered(Nm)' in df.columns:
            ax2b_twin.plot(plot_time, df['PI_P_Filtered(Nm)'], label='P Filtered', color='orange', linewidth=1.5, linestyle='-.', alpha=0.9)
        ax2b_twin.plot(plot_time, df['PI_P(Nm)'], label='P Final (Boosted)', color='green', linewidth=2)
        ax2b_twin.set_ylabel('P Term (Nm)', color='green')
    else:
        ax2b_twin.plot(plot_time, p_term, label='P Term', color='orange', linewidth=1.5)
    ax2b_twin.tick_params(axis='y', labelcolor='green')
    ax2b_twin.legend(loc='upper right', fontsize=9)

    if weight_add_time is not None:
        ax2b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax2b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    ax2b.set_title('DeltaF vs ADRC/PID Control Output')
    ax2b.legend(loc='upper left')
    ax2b.grid(True, alpha=0.3)
    
    # ---------- 图3: ADRC/PID控制项（扭矩） ----------
    ax3 = fig.add_subplot(gs[4, 1])
    if using_adrc_data:
        ax3.plot(plot_time, adrc_u0, label='ADRC u0', color='red', linewidth=1.5, linestyle='--')
        ax3.plot(plot_time, adrc_z2, label='ADRC z2 (Disturbance)', color='cyan', linewidth=1.5, linestyle=':')
        ax3.plot(plot_time, adrc_output, label='ADRC Output', color='green', linewidth=2)
        if adrc_p_gain is not None:
            ax3.plot(plot_time, adrc_p_gain, label='ADRC P Gain Mult', color='magenta', linewidth=1.5, linestyle='-.')
    elif 'PI_P(Nm)' in df.columns and 'PI_I(Nm)' in df.columns:
        torque_p = df['PI_P(Nm)']
        torque_i = df['PI_I(Nm)']
        if 'PI_P_Raw(Nm)' in df.columns:
            ax3.plot(plot_time, df['PI_P_Raw(Nm)'], label='P Raw', color='red', linewidth=1.5, linestyle='--', alpha=0.8)
        if 'PI_P_Filtered(Nm)' in df.columns:
            ax3.plot(plot_time, df['PI_P_Filtered(Nm)'], label='P Filtered', color='orange', linewidth=1.5, linestyle='-.', alpha=0.9)
        ax3.plot(plot_time, torque_p, label='P Final', color='green', linewidth=2)
        ax3.plot(plot_time, torque_i, label='I Term', color='cyan', linewidth=1.5)
        if 'PI_D(Nm)' in df.columns:
            ax3.plot(plot_time, df['PI_D(Nm)'], label='D Term', color='magenta', linewidth=1.5)
            pid_total = torque_p + torque_i + df['PI_D(Nm)']
        else:
            pid_total = torque_p + torque_i
        ax3.plot(plot_time, pid_total, label='PID Total', color='darkred', linewidth=2, linestyle='--')
    else:
        ax3.plot(plot_time, p_term, label='P Term', color='orange', linewidth=1.5)
        ax3.plot(plot_time, i_term, label='I Term', color='cyan', linewidth=1.5)
        ax3.plot(plot_time, pi_output, label='PI Output', color='red', linewidth=2, linestyle='--')
    ax3.axhline(y=0, color='k', linewidth=0.5)
    if weight_add_time is not None:
        ax3.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax3.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    ax3.set_ylabel('Torque (Nm)')
    ax3.set_title('ADRC/PID Control Terms (Torque)')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # ---------- 图4: 目标与实际扭矩 ----------
    ax4 = fig.add_subplot(gs[5, 0])
    if 'TargetTorque(Nm)' in df.columns and 'ActualTorque(Nm)' in df.columns:
        ax4.plot(plot_time, df['TargetTorque(Nm)'], label='Target Torque', color='red', linewidth=2, linestyle='--')
        ax4.plot(plot_time, df['ActualTorque(Nm)'], label='Actual Torque', color='blue', linewidth=1.5)
        ax4.axhline(y=0, color='gray', linestyle=':', alpha=0.5)
        if weight_add_time is not None:
            ax4.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
        if stable_time is not None:
            ax4.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
        ax4.set_ylabel('Torque (Nm)')
        ax4.set_title('Target vs Actual Torque')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
    else:
        ax4.plot(plot_time, current, label='Actual Current', color='orange', linewidth=1.5)
        ax4.plot(plot_time, target_current, label='Target Current', color='red', linewidth=1.5, linestyle='--')
        ax4.axhline(y=50, color='gray', linestyle=':', alpha=0.5, label='Base 50mA')
        if weight_add_time is not None:
            ax4.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
        if stable_time is not None:
            ax4.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
        ax4.set_ylabel('Current (mA)')
        ax4.set_title('Current Control')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
    
    # ---------- 图5: PI输出 vs 实际扭矩 ----------
    ax5 = fig.add_subplot(gs[5, 1])
    if 'ActualTorque(Nm)' in df.columns:
        if using_adrc_data:
            ax5.plot(plot_time, adrc_output, label='ADRC Output (Nm)', color='blue', linewidth=1.5)
            ax5.set_ylabel('ADRC Output (Nm)', color='blue')
        else:
            ax5.plot(plot_time, pi_output, label='PI Output (Nm)', color='blue', linewidth=1.5)
            ax5.set_ylabel('PI Output (Nm)', color='blue')
        ax5_twin = ax5.twinx()
        ax5_twin.plot(plot_time, df['ActualTorque(Nm)'], label='Actual Torque (Nm)', color='red', linewidth=1.5, linestyle='--')
        ax5_twin.set_ylabel('Actual Torque (Nm)', color='red')
        ax5_twin.tick_params(axis='y', labelcolor='red')
        ax5_twin.legend(loc='upper right')
        ax5.set_title('Control Output vs Actual Torque')
    else:
        ax5.plot(plot_time, pi_output, label='PI Output (Nm)', color='blue', linewidth=1.5)
        ax5_twin = ax5.twinx()
        ax5_twin.plot(plot_time, current-50, label='Current-50mA', color='red', linewidth=1.5, linestyle='--')
        ax5_twin.set_ylabel('Current Adjustment (mA)', color='red')
        ax5_twin.tick_params(axis='y', labelcolor='red')
        ax5_twin.legend(loc='upper right')
        ax5.set_title('PI Output vs Current Adjustment')
    ax5.tick_params(axis='y', labelcolor='blue')
    ax5.legend(loc='upper left')
    ax5.grid(True, alpha=0.3)
    
    # ---------- 图6: 电机速度 ----------
    ax6 = fig.add_subplot(gs[6, 0])
    ax6.plot(plot_time, motor_speed, label='Motor Speed', color='green', linewidth=1.5)
    if weight_add_time is not None:
        ax6.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax6.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    ax6.set_ylabel('Speed (rpm)')
    ax6.set_title('Motor Speed')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # ---------- 图7: 绳长 ----------
    ax7 = fig.add_subplot(gs[6, 1])
    ax7.plot(plot_time, position, label='Rope Length', color='brown', linewidth=2)
    if weight_add_time is not None:
        ax7.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax7.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    ax7.set_ylabel('Rope Length (mm)')
    ax7.set_xlabel('Time (s)')
    ax7.set_title('Rope Length')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    # ---------- 图7b: 重量数据 ----------
    ax7b = fig.add_subplot(gs[7, :])
    if has_weight_data:
        ax7b.plot(plot_time, weight_raw, label='Weight Raw', color='red', linewidth=1.5, linestyle='--', alpha=0.7)
        ax7b.plot(plot_time, weight_filtered, label='Weight Filtered', color='blue', linewidth=2)
        ax7b.axhline(y=f0, color='green', linestyle=':', linewidth=2, label=f'Target {f0:.2f}kg')
        ax7b.fill_between(plot_time, f0-0.5, f0+0.5, alpha=0.1, color='green', label='±0.5kg')
        if weight_add_time is not None:
            ax7b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
        if stable_time is not None:
            ax7b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
        ax7b.set_ylabel('Weight (kg)')
        ax7b.set_xlabel('Time (s)')
        ax7b.set_title('Weight Data (UART/TTL)')
        ax7b.legend(loc='best')
        ax7b.grid(True, alpha=0.3)
    else:
        ax7b.axis('off')
        ax7b.text(0.5, 0.5, 'No Weight Data', transform=ax7b.transAxes, ha='center', va='center', fontsize=12)
    
    # ---------- 图8: 绳子速度 ----------
    ax8 = fig.add_subplot(gs[8, 0])
    ax8.plot(plot_time, df['RopeVelocityRaw(m/s)'], label='Raw', alpha=0.5, linewidth=0.8)
    ax8.plot(plot_time, df['RopeVelocityFiltered(m/s)'], label='Filtered', linewidth=1.5)
    if weight_add_time is not None:
        ax8.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax8.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    ax8.set_ylabel('Velocity (m/s)')
    ax8.set_xlabel('Time (s)')
    ax8.set_title('Rope Velocity')
    ax8.legend()
    ax8.grid(True, alpha=0.3)
    
    # ---------- 图8b: 压力 vs 扭矩局部放大 ----------
    ax8b = fig.add_subplot(gs[8, 1])
    if 'ActualTorque(Nm)' in df.columns:
        actual_torque = df['ActualTorque(Nm)']
        torque_diff = np.diff(actual_torque)
        if len(torque_diff) > 0 and np.abs(torque_diff).max() > 1e-6:
            max_rise_idx = np.argmax(np.abs(torque_diff))
            start = max(0, max_rise_idx - 15)
            end = min(len(df)-1, max_rise_idx + 25)
            zoom_df = df.iloc[start:end]
            time_ms = (plot_time.iloc[start:end] - plot_time.iloc[start]) * 1000
            ax8b.plot(time_ms, pressure.iloc[start:end], label='Pressure', color='blue', linewidth=2.5, marker='o', markersize=4)
            ax8b_twin = ax8b.twinx()
            ax8b_twin.plot(time_ms, actual_torque.iloc[start:end], label='Actual Torque', color='red', linewidth=2.5, linestyle='--', marker='s', markersize=4)
            ax8b.set_title(f'Pressure vs Torque (Zoom {time_ms.iloc[-1]-time_ms.iloc[0]:.0f}ms)')
            ax8b.set_xlabel('Time (ms)')
            ax8b.set_ylabel('Pressure (kg)', color='blue')
            ax8b_twin.set_ylabel('Torque (Nm)', color='red')
            ax8b.legend(loc='upper left')
            ax8b_twin.legend(loc='upper right')
            ax8b.grid(True, alpha=0.3)
            ax8b_twin.grid(False)
            # 计算延迟
            p_vals = pressure.iloc[start:end].values
            t_vals = actual_torque.iloc[start:end].values
            if len(p_vals) > 10:
                p50 = p_vals.min() + 0.5*(p_vals.max()-p_vals.min())
                t50 = t_vals.min() + 0.5*(t_vals.max()-t_vals.min())
                p_idx = np.argmin(np.abs(p_vals - p50))
                t_idx = np.argmin(np.abs(t_vals - t50))
                delay = time_ms.iloc[t_idx] - time_ms.iloc[p_idx]
                ax8b.text(0.05, 0.9, f'Delay: {delay:.1f}ms', transform=ax8b.transAxes, fontsize=9,
                          bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7))
        else:
            ax8b.plot(plot_time, pressure, label='Pressure', color='blue', linewidth=2)
            ax8b_twin = ax8b.twinx()
            ax8b_twin.plot(plot_time, actual_torque, label='Actual Torque', color='red', linewidth=2, linestyle='--')
            ax8b.set_title('Pressure vs Torque (Full)')
            ax8b.set_xlabel('Time (s)')
            ax8b.set_ylabel('Pressure (kg)', color='blue')
            ax8b_twin.set_ylabel('Torque (Nm)', color='red')
            ax8b.legend(loc='upper left')
            ax8b_twin.legend(loc='upper right')
            ax8b.grid(True, alpha=0.3)
    else:
        # 回退到电流
        ax8b.plot(plot_time, pressure, label='Pressure', color='blue', linewidth=2)
        ax8b_twin = ax8b.twinx()
        ax8b_twin.plot(plot_time, current, label='Current (mA)', color='red', linewidth=2, linestyle='--')
        ax8b.set_title('Pressure vs Current')
        ax8b.set_xlabel('Time (s)')
        ax8b.set_ylabel('Pressure (kg)', color='blue')
        ax8b_twin.set_ylabel('Current (mA)', color='red')
        ax8b.legend(loc='upper left')
        ax8b_twin.legend(loc='upper right')
        ax8b.grid(True, alpha=0.3)
        if weight_add_time is not None:
            ax8b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
        if stable_time is not None:
            ax8b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    
    # ---------- 图9: 压力误差 ----------
    ax9 = fig.add_subplot(gs[9, 0])
    pressure_error = pressure - f0
    ax9.plot(plot_time, pressure_error, label='Pressure Error', color='red', linewidth=1.5)
    ax9.axhline(y=0, color='k', linewidth=0.5)
    ax9.axhline(y=0.5, color='g', linestyle='--', linewidth=1, alpha=0.5)
    ax9.axhline(y=-0.5, color='g', linestyle='--', linewidth=1, alpha=0.5)
    ax9.fill_between(plot_time, -0.5, 0.5, alpha=0.1, color='green')
    if weight_add_time is not None:
        ax9.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2)
    if stable_time is not None:
        ax9.axvline(x=stable_time, color='green', linestyle='--', linewidth=2)
    ax9.set_ylabel('Error (kg)')
    ax9.set_xlabel('Time (s)')
    ax9.set_title('Pressure Control Error')
    ax9.legend()
    ax9.grid(True, alpha=0.3)
    
    # ---------- 图9b: 扭矩/电流误差 ----------
    ax9b = fig.add_subplot(gs[9, 1])
    if 'TargetTorque(Nm)' in df.columns and 'ActualTorque(Nm)' in df.columns:
        torque_error = df['TargetTorque(Nm)'] - df['ActualTorque(Nm)']
        ax9b.plot(plot_time, torque_error, label='Torque Error', color='purple', linewidth=1.5)
        ax9b.axhline(y=0, color='k', linewidth=0.5)
        ax9b.axhline(y=torque_error.mean(), color='blue', linestyle='--', alpha=0.7, label=f'Mean {torque_error.mean():.3f}Nm')
        ax9b.fill_between(plot_time, torque_error, alpha=0.3, color='purple')
        ax9b.set_ylabel('Torque Error (Nm)')
        ax9b.set_title('Torque Tracking Error')
    else:
        current_error_abs = target_current - current
        ax9b.plot(plot_time, current_error_abs, label='Current Error', color='purple', linewidth=1.5)
        ax9b.axhline(y=0, color='k', linewidth=0.5)
        ax9b.axhline(y=current_error_abs.mean(), color='blue', linestyle='--', alpha=0.7, label=f'Mean {current_error_abs.mean():.1f}mA')
        ax9b.fill_between(plot_time, current_error_abs, alpha=0.3, color='purple')
        ax9b.set_ylabel('Current Error (mA)')
        ax9b.set_title('Current Tracking Error')
    ax9b.set_xlabel('Time (s)')
    ax9b.legend()
    ax9b.grid(True, alpha=0.3)
    if weight_add_time is not None:
        ax9b.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7)
    if stable_time is not None:
        ax9b.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7)
    
    # ---------- 图10: 实际扭矩放大 ----------
    ax10 = fig.add_subplot(gs[10, :])
    if 'ActualTorque(Nm)' in df.columns and 'TargetTorque(Nm)' in df.columns:
        ax10.plot(plot_time, df['ActualTorque(Nm)'], label='Actual Torque', color='darkblue', linewidth=2)
        ax10.plot(plot_time, df['TargetTorque(Nm)'], label='Target Torque', color='orange', linewidth=1.5, linestyle='--')
        ax10.axhline(y=0, color='gray', linestyle=':', alpha=0.7)
        if weight_add_time is not None:
            ax10.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, label='Weight added')
        if stable_time is not None:
            ax10.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, label='Stable')
        ax10.set_ylabel('Torque (Nm)')
        ax10.set_xlabel('Time (s)')
        ax10.set_title('Torque (Zoomed Y)')
        tmin, tmax = df['ActualTorque(Nm)'].min(), df['ActualTorque(Nm)'].max()
        margin = max(0.02, (tmax-tmin)*0.1)
        ax10.set_ylim([tmin-margin, tmax+margin])
        ax10.legend()
        ax10.grid(True, alpha=0.3)
    else:
        ax10.plot(plot_time, current, label='Actual Current', color='darkblue', linewidth=2)
        ax10.axhline(y=50, color='gray', linestyle=':', alpha=0.7, label='Base 50mA')
        if weight_add_time is not None:
            ax10.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, label='Weight added')
        if stable_time is not None:
            ax10.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, label='Stable')
        ax10.set_ylabel('Current (mA)')
        ax10.set_xlabel('Time (s)')
        ax10.set_title('Current (Zoomed Y)')
        cmin, cmax = current.min(), current.max()
        margin = max(2.0, (cmax-cmin)*0.1)
        ax10.set_ylim([cmin-margin, cmax+margin])
        ax10.legend()
        ax10.grid(True, alpha=0.3)
    
    # ==================== 图11（三轴修改） ====================
    ax11 = fig.add_subplot(gs[11:13, :])  # 跨两行
    
    # 左轴：压力
    ax11.plot(plot_time, pressure, label='Pressure', color='purple', linewidth=2)
    ax11.axhline(y=f0, color='r', linestyle='--', linewidth=1.5, alpha=0.7, label=f'Target F0={f0:.2f}kg')
    range_half = max(1.0, (pressure.max() - pressure.min()) / 2 * 1.2)
    ax11.set_ylim([f0 - range_half, f0 + range_half])
    ax11.set_ylabel('Pressure (kg)', color='purple', fontsize=12)
    ax11.tick_params(axis='y', labelcolor='purple')
    
    # 右轴1：绳长
    ax11_right1 = ax11.twinx()
    ax11_right1.plot(plot_time, position, label='Rope Length (mm)', color='teal', linewidth=2, linestyle='--')
    ax11_right1.set_ylabel('Rope Length (mm)', color='teal', fontsize=12)
    ax11_right1.tick_params(axis='y', labelcolor='teal')
    
    # 右轴2：重量（偏移）
    ax11_right2 = ax11.twinx()
    ax11_right2.spines['right'].set_position(('outward', 60))
    if has_weight_data:
        ax11_right2.plot(plot_time, weight_raw, label='Weight Raw', color='red', linewidth=1.5, linestyle='--', alpha=0.7)
        ax11_right2.plot(plot_time, weight_filtered, label='Weight Filtered', color='blue', linewidth=2)
    ax11_right2.set_ylabel('Weight (kg)', color='blue', fontsize=12)
    ax11_right2.tick_params(axis='y', labelcolor='blue')
    
    # 标记关键时间
    if weight_add_time is not None:
        ax11.axvline(x=weight_add_time, color='orange', linestyle='--', linewidth=2, alpha=0.7, label='Weight Added')
    if stable_time is not None:
        ax11.axvline(x=stable_time, color='green', linestyle='--', linewidth=2, alpha=0.7, label='Stable')
    
    # 合并图例
    lines1, labels1 = ax11.get_legend_handles_labels()
    lines2, labels2 = ax11_right1.get_legend_handles_labels()
    lines3, labels3 = ax11_right2.get_legend_handles_labels()
    ax11.legend(lines1 + lines2 + lines3, labels1 + labels2 + labels3, loc='upper right', fontsize=10)
    
    ax11.set_xlabel('Time (s)')
    ax11.set_title('Pressure (Left), Rope Length (Right1), Weight (Right2)', fontsize=13, fontweight='bold')
    ax11.grid(True, alpha=0.3)
    
    plt.tight_layout()
    base_name = os.path.basename(csv_file).replace('.csv', '_performance_analysis.png')
    output_file = os.path.join(os.getcwd(), base_name)
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n总览图表已保存: {output_file}")
    plt.close()
    
    # 总结输出
    print("\n" + "=" * 80)
    print("【算法性能总结】")
    print("=" * 80)
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
        print("  sample_period_ms: 采样周期（毫秒），默认自动检测")
        print("  自动检测：根据CSV中Time列的真实时间戳计算采样率")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    sample_period_ms = None
    if len(sys.argv) >= 3:
        sample_period_ms = int(sys.argv[2])
        print(f"使用指定采样周期: {sample_period_ms}ms ({int(1000/sample_period_ms)}Hz)")
    
    analyze_gravity_performance(csv_file, sample_period_ms)