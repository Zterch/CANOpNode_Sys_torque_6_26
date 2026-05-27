#!/usr/bin/env python3
"""
分析当前陷波滤波器的实际性能
当前参数：中心频率11Hz，极点半径0.9950
"""

import numpy as np
import matplotlib.pyplot as plt

# 当前陷波滤波器参数
fs = 50.0
f0 = 11.0
r = 0.9950

# 计算滤波器系数
w0 = 2 * np.pi * f0 / fs
b0 = 1.0
b1 = -2 * np.cos(w0)
b2 = 1.0
a0 = 1.0
a1 = -2 * r * np.cos(w0)
a2 = r * r

# 归一化
dc_gain = (b0 + b1 + b2) / (a0 + a1 + a2)
b0 /= dc_gain
b1 /= dc_gain
b2 /= dc_gain

# 计算频率响应
worN = 8192
w = np.linspace(0, np.pi, worN)
z = np.exp(1j * w)
num = b0 + b1 * z**(-1) + b2 * z**(-2)
den = a0 + a1 * z**(-1) + a2 * z**(-2)
h = num / den
freqs = w * fs / (2 * np.pi)

mag = np.abs(h)
mag_db = 20 * np.log10(mag + 1e-10)
phase_deg = np.angle(h) * 180 / np.pi

# 分析关键频率
def get_response(freq):
    idx = np.argmin(np.abs(freqs - freq))
    return mag[idx], mag_db[idx], phase_deg[idx]

print("=" * 80)
print("当前陷波滤波器性能分析")
print("=" * 80)
print(f"参数：中心频率 {f0}Hz，极点半径 {r}")
print(f"系数：B=[{b0:.6f}, {b1:.6f}, {b2:.6f}]")
print(f"      A=[{a0:.6f}, {a1:.6f}, {a2:.6f}]")

print(f"\n【各频率响应】")
for freq in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 20, 25]:
    m, m_db, p = get_response(freq)
    attenuation = (1 - m) * 100
    print(f"  {freq:2d}Hz: 幅值={m:.4f} ({m_db:+7.2f}dB), 衰减={attenuation:6.2f}%, 相移={p:+7.2f}°")

# 关键指标
mag_8, _, phase_8 = get_response(8)
mag_10, _, _ = get_response(10)
mag_11, _, _ = get_response(11)
mag_12, _, _ = get_response(12)

attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100

print(f"\n【关键性能指标】")
print(f"  10-12Hz最小衰减: {attenuation_10_12:.2f}%")
print(f"  11Hz中心衰减: {(1-mag_11)*100:.2f}% ({20*np.log10(mag_11+1e-10):.2f}dB)")
print(f"  8Hz相移: {phase_8:+.2f}°")
print(f"  8Hz幅值变化: {(mag_8-1)*100:+.2f}%")

# 带宽计算
# 找到-3dB点
idx_center = np.argmin(np.abs(freqs - f0))
idx_left = idx_center
idx_right = idx_center

while idx_left > 0 and mag_db[idx_left] < -3:
    idx_left -= 1
while idx_right < len(mag_db) - 1 and mag_db[idx_right] < -3:
    idx_right += 1

bandwidth_3db = freqs[idx_right] - freqs[idx_left]
print(f"\n【带宽】")
print(f"  -3dB带宽: {bandwidth_3db:.2f}Hz")
print(f"  品质因数Q: {f0/bandwidth_3db:.2f}")

# 绘制频率响应
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

ax1 = axes[0]
ax1.plot(freqs, mag_db, 'b-', linewidth=2)
ax1.axvline(x=f0, color='r', linestyle='--', label=f'Notch freq = {f0}Hz')
ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
ax1.axhline(y=-3, color='gray', linestyle='--', alpha=0.5, label='-3dB')
ax1.axhline(y=-14, color='red', linestyle='--', alpha=0.5, label='-14dB (80% attenuation)')
ax1.set_xlabel('Frequency (Hz)')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title(f'Current Notch Filter Response (r={r}, f0={f0}Hz)')
ax1.legend(loc='lower left', fontsize=8)
ax1.grid(True, alpha=0.3)
ax1.set_xlim([0, 25])
ax1.set_ylim([-50, 5])

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
plt.savefig('current_notch_analysis.png', dpi=150, bbox_inches='tight')
print(f"\n分析图已保存: current_notch_analysis.png")

print(f"\n【结论】")
print(f"  当前陷波滤波器在11Hz处有极深的陷波（{(1-mag_11)*100:.1f}%衰减）")
print(f"  但陷波带宽很窄（{bandwidth_3db:.2f}Hz），对10Hz和12Hz的衰减有限")
print(f"  8Hz信号几乎不受影响（相移{phase_8:.1f}°，幅值变化{(mag_8-1)*100:.1f}%）")
