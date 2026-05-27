#!/usr/bin/env python3
"""
陷波滤波器设计工具 - 8Hz版本 V2
更宽的陷波带宽，确保10-12Hz也有足够衰减
"""

import numpy as np
import matplotlib.pyplot as plt

def design_notch_filter(fs, f0, r):
    """设计二阶陷波滤波器"""
    w0 = 2 * np.pi * f0 / fs
    
    # 零点系数 (单位圆上)
    b0 = 1.0
    b1 = -2 * np.cos(w0)
    b2 = 1.0
    
    # 极点系数 (半径为r的圆上)
    a0 = 1.0
    a1 = -2 * r * np.cos(w0)
    a2 = r * r
    
    # 归一化直流增益为1
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

# 设计参数 - 9Hz陷波（10-12Hz中间）
fs = 50.0
f0 = 9.0  # 陷波中心频率
r = 0.90  # 更小的极点半径 = 更宽的陷波带宽

b, a = design_notch_filter(fs, f0, r)

# 计算频率响应
freqs, h = freqz(b, a, fs=fs)
mag_db = 20 * np.log10(np.abs(h) + 1e-10)
phase_deg = np.angle(h) * 180 / np.pi

# 分析性能
def get_response(freq):
    idx = np.argmin(np.abs(freqs - freq))
    mag = np.abs(h[idx])
    mag_db = 20 * np.log10(mag + 1e-10)
    phase = np.angle(h[idx]) * 180 / np.pi
    return mag, mag_db, phase

print("=" * 80)
print(f"9Hz陷波滤波器设计 - 极点半径 r = {r} (宽陷波带宽)")
print("=" * 80)

# 检查关键频率
for freq in [5, 6, 7, 8, 9, 10, 11, 12, 15]:
    mag, mag_db_val, phase = get_response(freq)
    attenuation = (1 - mag) * 100
    print(f"  {freq:2d}Hz: 幅值={mag:.4f} ({mag_db_val:+.2f}dB), 衰减={attenuation:.1f}%, 相移={phase:+.2f}°")

# 检查是否满足要求
mag_8, _, phase_8 = get_response(8)
mag_10, _, _ = get_response(10)
mag_11, _, _ = get_response(11)
mag_12, _, _ = get_response(12)

attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100

print(f"\n【性能评估】")
print(f"  10-12Hz最小衰减: {attenuation_10_12:.1f}% {'✓' if attenuation_10_12 > 80 else '✗'} (要求>80%)")
print(f"  8Hz相移: {abs(phase_8):.1f}° {'✓' if abs(phase_8) < 30 else '✗'} (要求<30°)")
print(f"  8Hz幅值变化: {abs(mag_8-1)*100:.1f}% {'✓' if abs(mag_8-1) < 0.2 else '✗'} (要求<20%)")

# 绘制频率响应
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

# 幅频响应
ax1 = axes[0]
ax1.plot(freqs, mag_db, 'b-', linewidth=2)
ax1.axvline(x=f0, color='r', linestyle='--', label=f'Notch freq = {f0}Hz')
ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
ax1.axhline(y=-14, color='r', linestyle='--', alpha=0.5, label='-14dB (80% attenuation)')
ax1.set_xlabel('Frequency (Hz)')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title(f'Notch Filter Frequency Response (r={r:.4f}, f0={f0}Hz)')
ax1.legend(loc='lower left')
ax1.grid(True, alpha=0.3)
ax1.set_xlim([0, 25])
ax1.set_ylim([-40, 5])

# 相频响应
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
plt.savefig('notch_filter_9hz_v2_design.png', dpi=150, bbox_inches='tight')
print(f"\n滤波器设计图已保存: notch_filter_9hz_v2_design.png")

# 输出C语言代码
print(f"\n{'='*80}")
print("C语言实现代码 - 9Hz陷波滤波器 (宽陷波)")
print(f"{'='*80}")
print(f"""
/* 陷波滤波器参数 - 中心频率{f0}Hz，极点半径{r:.4f}
 * 采样率: {fs}Hz
 * 宽陷波带宽，确保10-12Hz有足够衰减
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
