#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Torque Response Analysis Script
For analyzing gravity unload system torque control performance

Usage:
1. Direct run: python3 plot_torque_response.py <csv_file_path>
   Example: python3 plot_torque_response.py /home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys_torque_6_15/logdata/gravity_data_20260615_185607.csv

2. Auto-find latest file: python3 plot_torque_response.py

3. Output includes:
   - Target vs Actual Torque curves
   - Torque tracking error curve
   - Motor speed response
   - Performance metrics (max error, RMS error, response delay, etc.)
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from glob import glob
from scipy.signal import find_peaks

# Font settings - use English to avoid encoding issues
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

def find_latest_csv(log_dir="/home/zterch/VS_Project/Nimo_COp_Prj/CANOpNode_Sys_torque_6_15/logdata"):
    """Find the latest CSV file"""
    csv_files = glob(os.path.join(log_dir, "gravity_data_*.csv"))
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getmtime)

def load_data(csv_file):
    """Load CSV data"""
    print(f"Loading data: {csv_file}")
    df = pd.read_csv(csv_file)
    # Convert to seconds (100Hz sampling, 10ms per row)
    df['Time_sec'] = df.index * 0.01
    return df

def calculate_response_delay(time, target, actual, threshold=0.1):
    """
    Calculate response delay using cross-correlation
    Returns: delay in milliseconds
    """
    # Find significant changes in target
    target_diff = np.diff(target)
    change_indices = np.where(np.abs(target_diff) > threshold)[0]
    
    if len(change_indices) < 2:
        return None
    
    delays = []
    for idx in change_indices[:10]:  # Check first 10 changes
        # Get window around change
        start_idx = max(0, idx - 10)
        end_idx = min(len(target), idx + 50)
        
        target_window = target[start_idx:end_idx]
        actual_window = actual[start_idx:end_idx]
        
        if len(target_window) < 20:
            continue
        
        # Cross-correlation to find delay
        correlation = np.correlate(target_window - np.mean(target_window),
                                   actual_window - np.mean(actual_window),
                                   mode='full')
        lag = np.argmax(correlation) - (len(target_window) - 1)
        delay_ms = lag * 10  # 100Hz = 10ms per sample
        
        if 0 < delay_ms < 500:  # Reasonable delay range
            delays.append(delay_ms)
    
    return np.mean(delays) if delays else None

def calculate_rise_time(time, target, actual, threshold_low=0.1, threshold_high=0.9):
    """
    Calculate rise time (10% to 90% of step response)
    Returns: average rise time in milliseconds
    """
    # Find step changes in target
    target_diff = np.diff(target)
    step_indices = np.where(np.abs(target_diff) > 0.05)[0]
    
    rise_times = []
    for idx in step_indices:
        if idx + 50 >= len(actual):
            continue
        
        step_size = target_diff[idx]
        if abs(step_size) < 0.05:
            continue
        
        start_val = target[idx]
        end_val = target[idx + 1]
        
        # Find 10% and 90% points in actual response
        low_threshold = start_val + 0.1 * (end_val - start_val)
        high_threshold = start_val + 0.9 * (end_val - start_val)
        
        actual_window = actual[idx:idx+50]
        
        # Find crossing points
        low_cross = None
        high_cross = None
        for i, val in enumerate(actual_window):
            if low_cross is None and ((step_size > 0 and val >= low_threshold) or 
                                       (step_size < 0 and val <= low_threshold)):
                low_cross = i
            if high_cross is None and ((step_size > 0 and val >= high_threshold) or 
                                        (step_size < 0 and val <= high_threshold)):
                high_cross = i
                break
        
        if low_cross is not None and high_cross is not None and high_cross > low_cross:
            rise_time = (high_cross - low_cross) * 10  # 10ms per sample
            if 10 < rise_time < 500:
                rise_times.append(rise_time)
    
    return np.mean(rise_times) if rise_times else None

def calculate_metrics(df):
    """Calculate performance metrics"""
    mask = df['TargetTorque(Nm)'] != 0
    if mask.sum() == 0:
        return None
    
    target = df.loc[mask, 'TargetTorque(Nm)']
    actual = df.loc[mask, 'ActualTorque(Nm)']
    error = target - actual
    time = df.loc[mask, 'Time_sec']
    
    metrics = {
        'max_error': np.max(np.abs(error)),
        'mean_error': np.mean(error),
        'rms_error': np.sqrt(np.mean(error**2)),
        'std_error': np.std(error),
        'max_target': np.max(target),
        'max_actual': np.max(actual),
        'correlation': np.corrcoef(target, actual)[0, 1],
        'response_delay_ms': calculate_response_delay(time.values, target.values, actual.values),
        'rise_time_ms': calculate_rise_time(time.values, target.values, actual.values)
    }
    
    return metrics

def plot_torque_response(df, save_path=None):
    """Plot torque response curves"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Torque Control Response Analysis', fontsize=16, fontweight='bold')
    
    time = df['Time_sec']
    target = df['TargetTorque(Nm)']
    actual = df['ActualTorque(Nm)']
    error = target - actual
    
    # Plot 1: Target vs Actual Torque
    ax1 = axes[0]
    ax1.plot(time, target, 'b-', label='Target Torque', linewidth=1.5, alpha=0.8)
    ax1.plot(time, actual, 'r-', label='Actual Torque', linewidth=1.5, alpha=0.8)
    ax1.set_ylabel('Torque (Nm)', fontsize=11)
    ax1.set_title('Target vs Actual Torque', fontsize=12)
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([time.iloc[0], time.iloc[-1]])
    
    # Plot 2: Tracking Error
    ax2 = axes[1]
    ax2.plot(time, error, 'g-', label='Tracking Error', linewidth=1.2, alpha=0.8)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    ax2.fill_between(time, error, alpha=0.3, color='green')
    ax2.set_ylabel('Error (Nm)', fontsize=11)
    ax2.set_title('Torque Tracking Error', fontsize=12)
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([time.iloc[0], time.iloc[-1]])
    
    # Plot 3: Motor Speed
    ax3 = axes[2]
    if 'MotorSpeed(rpm)' in df.columns:
        speed = df['MotorSpeed(rpm)']
        ax3.plot(time, speed, 'm-', label='Motor Speed', linewidth=1.2, alpha=0.8)
        ax3.set_ylabel('Speed (rpm)', fontsize=11)
        ax3.set_title('Motor Speed Response', fontsize=12)
        ax3.legend(loc='upper right', fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlabel('Time (sec)', fontsize=11)
    ax3.set_xlim([time.iloc[0], time.iloc[-1]])
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved: {save_path}")
    
    return fig

def print_metrics(metrics):
    """Print performance metrics"""
    if metrics is None:
        print("\nNo torque control data detected")
        return
    
    print("\n" + "="*60)
    print("Torque Control Performance Metrics")
    print("="*60)
    print(f"Max Tracking Error:     {metrics['max_error']:.4f} Nm")
    print(f"Mean Tracking Error:    {metrics['mean_error']:.4f} Nm")
    print(f"RMS Error:              {metrics['rms_error']:.4f} Nm")
    print(f"Std Dev of Error:       {metrics['std_error']:.4f} Nm")
    print(f"Max Target Torque:      {metrics['max_target']:.4f} Nm")
    print(f"Max Actual Torque:      {metrics['max_actual']:.4f} Nm")
    print(f"Correlation Coeff:      {metrics['correlation']:.4f}")
    
    if metrics['response_delay_ms'] is not None:
        print(f"Response Delay:         {metrics['response_delay_ms']:.1f} ms")
    else:
        print(f"Response Delay:         N/A")
    
    if metrics['rise_time_ms'] is not None:
        print(f"Rise Time (10%-90%):    {metrics['rise_time_ms']:.1f} ms")
    else:
        print(f"Rise Time (10%-90%):    N/A")
    
    print("="*60)

def main():
    # Get CSV file path
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        print("No CSV file specified, finding latest data file...")
        csv_file = find_latest_csv()
        if csv_file is None:
            print("Error: No CSV data file found")
            print(f"Usage: python3 {sys.argv[0]} <csv_file_path>")
            sys.exit(1)
        print(f"Found: {csv_file}")
    
    # Check file exists
    if not os.path.exists(csv_file):
        print(f"Error: File not found: {csv_file}")
        sys.exit(1)
    
    # Load data
    try:
        df = load_data(csv_file)
    except Exception as e:
        print(f"Failed to load data: {e}")
        sys.exit(1)
    
    # Check required columns
    required_cols = ['TargetTorque(Nm)', 'ActualTorque(Nm)']
    for col in required_cols:
        if col not in df.columns:
            print(f"Error: CSV file missing required column: {col}")
            sys.exit(1)
    
    # Calculate metrics
    metrics = calculate_metrics(df)
    print_metrics(metrics)
    
    # Generate save path
    base_name = os.path.splitext(csv_file)[0]
    save_path = base_name + "_torque_response.png"
    
    # Plot
    fig = plot_torque_response(df, save_path)
    plt.close()
    
    print(f"\nAnalysis complete!")
    print(f"Data file: {csv_file}")
    print(f"Plot file: {save_path}")

if __name__ == "__main__":
    main()
