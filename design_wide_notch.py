#!/usr/bin/env python3
"""
宽陷波滤波器设计 - 针对锤击测试优化
目标：在50Hz采样率下，尽可能满足：
- 10-12Hz衰减尽可能大
- 8Hz相移<30°
- 8Hz幅值变化<20%

采用极点半径r=0.90，获得更宽的陷波带宽
"""

import numpy as np
import matplotlib.pyplot as plt

def design_notch_filter(fs, f0, r):
    """设计二阶陷波滤波器"""
    w0 = 2 * np.pi * f0 / fs
    
    b0 = 1.0
    b1 = -2 * np.cos(w0)
    b2 = 1.0
    
    a0 = 1.0
    a1 = -2 * r * np.cos(w0)
    a2 = r * r
    
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

# 设计参数 - 使用更小的极点半径获得更宽的陷波
fs = 50.0
f0 = 11.0  # 陷波中心频率
r = 0.90   # 极点半径 - 更小的半径 = 更宽的陷波

b, a = design_notch_filter(fs, f0, r)
freqs, h = freqz(b, a, fs=fs)

mag = np.abs(h)
mag_db = 20 * np.log10(mag + 1e-10)
phase_deg = np.angle(h) * 180 / np.pi

def get_response(freq):
    idx = np.argmin(np.abs(freqs - freq))
    return mag[idx], mag_db[idx], phase_deg[idx]

print("=" * 80)
print(f"宽陷波滤波器设计 - 极点半径 r = {r}")
print("=" * 80)

print(f"\n【各频率响应】")
for freq in [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]:
    m, m_db, p = get_response(freq)
    attenuation = (1 - m) * 100
    print(f"  {freq:2d}Hz: 幅值={m:.4f} ({m_db:+7.2f}dB), 衰减={attenuation:6.1f}%, 相移={p:+7.2f}°")

mag_8, _, phase_8 = get_response(8)
mag_10, _, _ = get_response(10)
mag_11, _, _ = get_response(11)
mag_12, _, _ = get_response(12)

attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100

print(f"\n【关键性能指标】")
print(f"  10-12Hz最小衰减: {attenuation_10_12:.1f}%")
print(f"  11Hz中心衰减: {(1-mag_11)*100:.1f}% ({20*np.log10(mag_11+1e-10):.1f}dB)")
print(f"  8Hz相移: {phase_8:+.1f}° {'✓' if abs(phase_8) < 30 else '✗'} (要求<30°)")
print(f"  8Hz幅值变化: {(mag_8-1)*100:+.1f}% {'✓' if abs(mag_8-1) < 20 else '✗'} (要求<20%)")

# 绘制频率响应
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

ax1 = axes[0]
ax1.plot(freqs, mag_db, 'b-', linewidth=2)
ax1.axvline(x=f0, color='r', linestyle='--', label=f'Notch freq = {f0}Hz')
ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
ax1.axhline(y=-14, color='r', linestyle='--', alpha=0.5, label='-14dB (80% attenuation)')
ax1.axhline(y=-8, color='orange', linestyle='--', alpha=0.5, label='-8dB (60% attenuation)')
ax1.set_xlabel('Frequency (Hz)')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title(f'Wide Notch Filter Response (r={r}, f0={f0}Hz)')
ax1.legend(loc='lower left', fontsize=8)
ax1.grid(True, alpha=0.3)
ax1.set_xlim([0, 25])
ax1.set_ylim([-30, 5])

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
plt.savefig('wide_notch_design.png', dpi=150, bbox_inches='tight')
print(f"\n设计图已保存: wide_notch_design.png")

print(f"\n{'='*80}")
print("C语言实现代码 - 宽陷波滤波器")
print(f"{'='*80}")
print(f"""
/* 宽陷波滤波器参数 - 中心频率{f0}Hz，极点半径{r:.2f}
 * 采样率: {fs}Hz
 * 性能:
 *   - 10-12Hz衰减: {attenuation_10_12:.1f}%
 *   - 8Hz相移: {phase_8:+.1f}°
 *   - 8Hz幅值变化: {(mag_8-1)*100:+.1f}%
 */
#define PRESSURE_NOTCH_B0    {b[0]:.6f}f
#define PRESSURE_NOTCH_B1    {b[1]:.6f}f
#define PRESSURE_NOTCH_B2    {b[2]:.6f}f
#define PRESSURE_NOTCH_A0    {a[0]:.6f}f
#define PRESSURE_NOTCH_A1    {a[1]:.6f}f
#define PRESSURE_NOTCH_A2    {a[2]:.6f}f
""")
