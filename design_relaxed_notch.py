#!/usr/bin/env python3
"""
放宽要求的陷波滤波器设计
要求：
- 10-12Hz衰减 > 60% (>8dB)
- 8Hz相移 < 30°
- 8Hz幅值变化 < 20%
"""

import numpy as np
import matplotlib.pyplot as plt

def design_notch_filter(fs, f0, r):
    """设计二阶陷波滤波器"""
    w0 = 2 * np.pi * f0 / fs
    
    # 零点系数
    b0 = 1.0
    b1 = -2 * np.cos(w0)
    b2 = 1.0
    
    # 极点系数
    a0 = 1.0
    a1 = -2 * r * np.cos(w0)
    a2 = r * r
    
    # 归一化
    dc_gain = (b0 + b1 + b2) / (a0 + a1 + a2)
    b0 /= dc_gain
    b1 /= dc_gain
    b2 /= dc_gain
    
    return [b0, b1, b2], [a0, a1, a2]

def freqz(b, a, worN=8192, fs=1.0):
    """计算频率响应"""
    w = np.linspace(0, np.pi, worN)
    z = np.exp(1j * w)
    
    num = b[0] + b[1] * z**(-1) + b[2] * z**(-2)
    den = a[0] + a[1] * z**(-1) + a[2] * z**(-2)
    h = num / den
    
    freqs = w * fs / (2 * np.pi)
    return freqs, h

# 搜索最优参数
fs = 50.0
f0 = 11.0  # 陷波中心频率

print("=" * 80)
print("放宽要求的陷波滤波器设计")
print("=" * 80)
print(f"采样率: {fs}Hz")
print(f"陷波中心频率: {f0}Hz")
print("\n搜索最优极点半径...")

best_r = None
best_score = -1

for r in np.arange(0.70, 0.99, 0.01):
    b, a = design_notch_filter(fs, f0, r)
    freqs, h = freqz(b, a, fs=fs)
    
    # 找到关键频率的响应
    def get_response(freq):
        idx = np.argmin(np.abs(freqs - freq))
        mag = np.abs(h[idx])
        mag_db = 20 * np.log10(mag + 1e-10)
        phase = np.angle(h[idx]) * 180 / np.pi
        return mag, mag_db, phase
    
    mag_8, _, phase_8 = get_response(8)
    mag_10, _, _ = get_response(10)
    mag_11, _, _ = get_response(11)
    mag_12, _, _ = get_response(12)
    
    attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100
    
    # 检查是否满足放宽的要求
    meets_attenuation = attenuation_10_12 > 60
    meets_phase = abs(phase_8) < 30
    meets_magnitude = abs(mag_8 - 1.0) < 0.2
    
    if meets_attenuation and meets_phase and meets_magnitude:
        # 计算得分：在满足条件的情况下，衰减越大越好
        score = attenuation_10_12
        if score > best_score:
            best_score = score
            best_r = r

if best_r is not None:
    b, a = design_notch_filter(fs, f0, best_r)
    freqs, h = freqz(b, a, fs=fs)
    
    print(f"\n✓ 找到最优参数!")
    print(f"  极点半径 r = {best_r:.4f}")
    
    # 详细性能分析
    def get_response(freq):
        idx = np.argmin(np.abs(freqs - freq))
        mag = np.abs(h[idx])
        mag_db = 20 * np.log10(mag + 1e-10)
        phase = np.angle(h[idx]) * 180 / np.pi
        return mag, mag_db, phase
    
    print(f"\n【滤波器性能】")
    for freq in [5, 6, 7, 8, 9, 10, 11, 12, 15]:
        mag, mag_db_val, phase = get_response(freq)
        attenuation = (1 - mag) * 100
        print(f"  {freq:2d}Hz: 幅值={mag:.4f} ({mag_db_val:+.2f}dB), 衰减={attenuation:.1f}%, 相移={phase:+.2f}°")
    
    mag_8, _, phase_8 = get_response(8)
    mag_10, _, _ = get_response(10)
    mag_11, _, _ = get_response(11)
    mag_12, _, _ = get_response(12)
    
    attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100
    
    print(f"\n【性能评估（放宽要求）】")
    print(f"  10-12Hz最小衰减: {attenuation_10_12:.1f}% {'✓' if attenuation_10_12 > 60 else '✗'} (要求>60%)")
    print(f"  8Hz相移: {abs(phase_8):.1f}° {'✓' if abs(phase_8) < 30 else '✗'} (要求<30°)")
    print(f"  8Hz幅值变化: {abs(mag_8-1)*100:.1f}% {'✓' if abs(mag_8-1) < 20 else '✗'} (要求<20%)")
    
    # 绘制频率响应
    mag_db = 20 * np.log10(np.abs(h) + 1e-10)
    phase_deg = np.angle(h) * 180 / np.pi
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    ax1 = axes[0]
    ax1.plot(freqs, mag_db, 'b-', linewidth=2)
    ax1.axvline(x=f0, color='r', linestyle='--', label=f'Notch freq = {f0}Hz')
    ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
    ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
    ax1.axhline(y=-8, color='orange', linestyle='--', alpha=0.5, label='-8dB (60% attenuation)')
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title(f'Notch Filter Frequency Response (r={best_r:.4f}, f0={f0}Hz)')
    ax1.legend(loc='lower left')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 25])
    ax1.set_ylim([-20, 5])
    
    ax2 = axes[1]
    ax2.plot(freqs, phase_deg, 'b-', linewidth=2)
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
    plt.savefig('notch_filter_relaxed_design.png', dpi=150, bbox_inches='tight')
    print(f"\n滤波器设计图已保存: notch_filter_relaxed_design.png")
    
    # 输出C语言代码
    print(f"\n{'='*80}")
    print("C语言实现代码 - 放宽要求的陷波滤波器")
    print(f"{'='*80}")
    print(f"""
/* 陷波滤波器参数 - 中心频率{f0}Hz，极点半径{best_r:.4f}
 * 采样率: {fs}Hz
 * 性能（放宽要求）:
 *   - 10-12Hz衰减 > 60%
 *   - 8Hz相移 < 30°
 *   - 8Hz幅值变化 < 20%
 */
#define NOTCH_B0    {b[0]:.6f}f
#define NOTCH_B1    {b[1]:.6f}f
#define NOTCH_B2    {b[2]:.6f}f
#define NOTCH_A0    {a[0]:.6f}f
#define NOTCH_A1    {a[1]:.6f}f
#define NOTCH_A2    {a[2]:.6f}f

static float s_notch_x1 = 0.0f, s_notch_x2 = 0.0f;
static float s_notch_y1 = 0.0f, s_notch_y2 = 0.0f;
static int s_notch_initialized = 0;

float apply_notch_filter(float x0) {{
    if (!s_notch_initialized) {{
        s_notch_x1 = s_notch_x2 = x0;
        s_notch_y1 = s_notch_y2 = x0;
        s_notch_initialized = 1;
    }}
    
    float y0 = NOTCH_B0 * x0 + NOTCH_B1 * s_notch_x1 + NOTCH_B2 * s_notch_x2
               - NOTCH_A1 * s_notch_y1 - NOTCH_A2 * s_notch_y2;
    
    s_notch_x2 = s_notch_x1;
    s_notch_x1 = x0;
    s_notch_y2 = s_notch_y1;
    s_notch_y1 = y0;
    
    return y0;
}}
""")
else:
    print("\n✗ 未找到满足放宽要求的参数")
    print("建议：进一步放宽要求或改变滤波器类型")
