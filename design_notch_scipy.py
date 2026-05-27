#!/usr/bin/env python3
"""
使用SciPy设计陷波滤波器
要求：
- 10-12Hz衰减 > 70% (>10.5dB)
- 8Hz相移 < 30°
- 8Hz幅值变化 < 20%
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def design_and_analyze_notch(fs, f0, Q):
    """
    设计陷波滤波器并分析性能
    
    Args:
        fs: 采样频率 (Hz)
        f0: 陷波中心频率 (Hz)
        Q: 品质因数，Q = f0 / bandwidth
    """
    # 使用scipy设计陷波滤波器
    b, a = signal.iirnotch(f0, Q, fs)
    
    # 计算频率响应
    w, h = signal.freqz(b, a, worN=8192, fs=fs)
    
    # 计算幅度和相位
    mag = np.abs(h)
    mag_db = 20 * np.log10(mag + 1e-10)
    phase_deg = np.angle(h) * 180 / np.pi
    
    return b, a, w, mag, mag_db, phase_deg

def get_response_at_freq(w, mag, phase_deg, target_freq):
    """获取特定频率的响应"""
    idx = np.argmin(np.abs(w - target_freq))
    return mag[idx], mag_db[idx], phase_deg[idx]

def check_requirements(w, mag, phase_deg):
    """检查是否满足要求"""
    # 10-12Hz衰减
    mag_10, _, _ = get_response_at_freq(w, mag, phase_deg, 10)
    mag_11, _, _ = get_response_at_freq(w, mag, phase_deg, 11)
    mag_12, _, _ = get_response_at_freq(w, mag, phase_deg, 12)
    attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100
    
    # 8Hz指标
    mag_8, _, phase_8 = get_response_at_freq(w, mag, phase_deg, 8)
    
    meets_attenuation = attenuation_10_12 > 70
    meets_phase = abs(phase_8) < 30
    meets_magnitude = abs(mag_8 - 1.0) < 0.2
    
    return {
        'attenuation_10_12': attenuation_10_12,
        'mag_8': mag_8,
        'phase_8': phase_8,
        'meets_all': meets_attenuation and meets_phase and meets_magnitude,
        'meets_attenuation': meets_attenuation,
        'meets_phase': meets_phase,
        'meets_magnitude': meets_magnitude
    }

# 参数设置
fs = 50.0  # 采样频率
f0 = 11.0  # 陷波中心频率

print("=" * 80)
print("SciPy陷波滤波器设计 - 搜索最优Q值")
print("=" * 80)
print(f"采样率: {fs}Hz")
print(f"陷波中心频率: {f0}Hz")
print("\n搜索最优品质因数Q...")

best_Q = None
best_score = -1
results = []

# 搜索Q值范围
for Q in np.arange(0.5, 5.0, 0.1):
    b, a, w, mag, mag_db, phase_deg = design_and_analyze_notch(fs, f0, Q)
    result = check_requirements(w, mag, phase_deg)
    
    results.append({
        'Q': Q,
        'attenuation': result['attenuation_10_12'],
        'phase_8': result['phase_8'],
        'mag_8': result['mag_8'],
        'meets_all': result['meets_all']
    })
    
    if result['meets_all']:
        # 计算得分：在满足所有条件的情况下，衰减越大越好
        score = result['attenuation_10_12']
        if score > best_score:
            best_score = score
            best_Q = Q

# 打印搜索结果
print(f"\n{'Q值':>6} | {'10-12Hz衰减':>12} | {'8Hz相移':>10} | {'8Hz幅值':>10} | {'满足要求':>8}")
print("-" * 70)
for r in results:
    meets_mark = "✓" if r['meets_all'] else "✗"
    print(f"{r['Q']:6.1f} | {r['attenuation']:11.1f}% | {r['phase_8']:9.1f}° | {r['mag_8']:9.4f} | {meets_mark:>8}")

if best_Q is not None:
    print(f"\n✓ 找到最优参数!")
    print(f"  品质因数 Q = {best_Q:.2f}")
    
    # 详细分析
    b, a, w, mag, mag_db, phase_deg = design_and_analyze_notch(fs, f0, best_Q)
    
    print(f"\n【详细频率响应】")
    for freq in [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]:
        m, m_db, p = get_response_at_freq(w, mag, phase_deg, freq)
        attenuation = (1 - m) * 100
        print(f"  {freq:2d}Hz: 幅值={m:.4f} ({m_db:+7.2f}dB), 衰减={attenuation:6.1f}%, 相移={p:+7.2f}°")
    
    # 关键指标
    result = check_requirements(w, mag, phase_deg)
    mag_8, _, phase_8 = get_response_at_freq(w, mag, phase_deg, 8)
    
    print(f"\n【性能评估】")
    print(f"  10-12Hz最小衰减: {result['attenuation_10_12']:.1f}% {'✓' if result['meets_attenuation'] else '✗'} (要求>70%)")
    print(f"  8Hz相移: {abs(phase_8):.1f}° {'✓' if result['meets_phase'] else '✗'} (要求<30°)")
    print(f"  8Hz幅值变化: {abs(mag_8-1)*100:.1f}% {'✓' if result['meets_magnitude'] else '✗'} (要求<20%)")
    
    # 计算-3dB带宽
    idx_center = np.argmin(np.abs(w - f0))
    idx_left = idx_center
    idx_right = idx_center
    
    while idx_left > 0 and mag_db[idx_left] < -3:
        idx_left -= 1
    while idx_right < len(mag_db) - 1 and mag_db[idx_right] < -3:
        idx_right += 1
    
    bandwidth_3db = w[idx_right] - w[idx_left]
    print(f"\n  -3dB带宽: {bandwidth_3db:.2f}Hz")
    print(f"  实际Q值: {f0/bandwidth_3db:.2f}")
    
    # 绘制频率响应
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    ax1 = axes[0]
    ax1.plot(w, mag_db, 'b-', linewidth=2)
    ax1.axvline(x=f0, color='r', linestyle='--', label=f'Notch freq = {f0}Hz')
    ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
    ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
    ax1.axhline(y=-10.5, color='orange', linestyle='--', alpha=0.5, label='-10.5dB (70% attenuation)')
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title(f'Notch Filter Response (Q={best_Q:.2f}, f0={f0}Hz)')
    ax1.legend(loc='lower left', fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 25])
    ax1.set_ylim([-30, 5])
    
    ax2 = axes[1]
    ax2.plot(w, phase_deg, 'b-', linewidth=2)
    ax2.axvline(x=f0, color='r', linestyle='--')
    ax2.axhline(y=30, color='gray', linestyle='--', alpha=0.5, label='±30°')
    ax2.axhline(y=-30, color='gray', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Phase (degrees)')
    ax2.set_title('Phase Response')
    ax2.legend(loc='lower left')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 25])
    ax2.set_ylim([-90, 90])
    
    plt.tight_layout()
    plt.savefig('notch_filter_scipy_design.png', dpi=150, bbox_inches='tight')
    print(f"\n设计图已保存: notch_filter_scipy_design.png")
    
    # 输出C语言代码
    print(f"\n{'='*80}")
    print("滤波器系数 (b, a)")
    print(f"{'='*80}")
    print(f"b = [{', '.join([f'{x:.6f}' for x in b])}]")
    print(f"a = [{', '.join([f'{x:.6f}' for x in a])}]")
    
else:
    print("\n✗ 未找到满足所有条件的Q值")
    print("建议：进一步放宽要求或考虑其他滤波器类型")
