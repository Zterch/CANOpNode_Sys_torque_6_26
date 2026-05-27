#!/usr/bin/env python3
"""
级联滤波器设计工具
第一级：低通滤波器（截止频率15Hz，去除高频噪声）
第二级：陷波滤波器（中心频率11Hz，去除10-12Hz干扰）
"""

import numpy as np
import matplotlib.pyplot as plt

def design_lowpass_filter(fs, fc):
    """设计一阶低通滤波器"""
    # 一阶IIR低通滤波器
    # y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    # alpha = 1 / (1 + 2*pi*fc/fs)
    alpha = 1.0 / (1.0 + 2.0 * np.pi * fc / fs)
    return alpha

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

def apply_lowpass(x, alpha, state):
    """应用一阶低通滤波器"""
    y = alpha * x + (1 - alpha) * state
    return y, y

def apply_notch(x, b, a, x_hist, y_hist):
    """应用二阶陷波滤波器"""
    y = b[0] * x + b[1] * x_hist[0] + b[2] * x_hist[1] - a[1] * y_hist[0] - a[2] * y_hist[1]
    x_hist[1] = x_hist[0]
    x_hist[0] = x
    y_hist[1] = y_hist[0]
    y_hist[0] = y
    return y

# 设计参数
fs = 50.0
fc_lp = 15.0  # 低通截止频率
f0_notch = 11.0  # 陷波中心频率
r_notch = 0.98  # 陷波极点半径

# 设计滤波器
alpha_lp = design_lowpass_filter(fs, fc_lp)
b_notch, a_notch = design_notch_filter(fs, f0_notch, r_notch)

print("=" * 80)
print("级联滤波器设计")
print("=" * 80)
print(f"第一级：一阶低通滤波器，截止频率 {fc_lp}Hz，alpha = {alpha_lp:.4f}")
print(f"第二级：二阶陷波滤波器，中心频率 {f0_notch}Hz，极点半径 {r_notch}")

# 计算级联频率响应
def cascaded_freqz(alpha_lp, b_notch, a_notch, worN=8192, fs=1.0):
    """计算级联滤波器的频率响应"""
    w = np.linspace(0, np.pi, worN)
    z = np.exp(1j * w)
    
    # 低通滤波器响应
    # H_lp(z) = alpha / (1 - (1-alpha)*z^-1)
    h_lp = alpha_lp / (1 - (1 - alpha_lp) * z**(-1))
    
    # 陷波滤波器响应
    num_notch = b_notch[0] + b_notch[1] * z**(-1) + b_notch[2] * z**(-2)
    den_notch = a_notch[0] + a_notch[1] * z**(-1) + a_notch[2] * z**(-2)
    h_notch = num_notch / den_notch
    
    # 级联响应
    h_cascaded = h_lp * h_notch
    
    freqs = w * fs / (2 * np.pi)
    return freqs, h_lp, h_notch, h_cascaded

freqs, h_lp, h_notch, h_cascaded = cascaded_freqz(alpha_lp, b_notch, a_notch, fs=fs)

mag_lp_db = 20 * np.log10(np.abs(h_lp) + 1e-10)
mag_notch_db = 20 * np.log10(np.abs(h_notch) + 1e-10)
mag_cascaded_db = 20 * np.log10(np.abs(h_cascaded) + 1e-10)

phase_cascaded_deg = np.angle(h_cascaded) * 180 / np.pi

# 分析性能
def get_response(h, freq, freqs):
    idx = np.argmin(np.abs(freqs - freq))
    mag = np.abs(h[idx])
    mag_db = 20 * np.log10(mag + 1e-10)
    phase = np.angle(h[idx]) * 180 / np.pi
    return mag, mag_db, phase

print("\n【级联滤波器性能】")
for freq in [5, 6, 7, 8, 9, 10, 11, 12, 15, 20]:
    mag, mag_db_val, phase = get_response(h_cascaded, freq, freqs)
    attenuation = (1 - mag) * 100
    print(f"  {freq:2d}Hz: 幅值={mag:.4f} ({mag_db_val:+.2f}dB), 衰减={attenuation:.1f}%, 相移={phase:+.2f}°")

# 检查是否满足要求
mag_8, _, phase_8 = get_response(h_cascaded, 8, freqs)
mag_10, _, _ = get_response(h_cascaded, 10, freqs)
mag_11, _, _ = get_response(h_cascaded, 11, freqs)
mag_12, _, _ = get_response(h_cascaded, 12, freqs)

attenuation_10_12 = (1 - max(mag_10, mag_11, mag_12)) * 100

print(f"\n【性能评估】")
print(f"  10-12Hz最小衰减: {attenuation_10_12:.1f}% {'✓' if attenuation_10_12 > 80 else '✗'} (要求>80%)")
print(f"  8Hz相移: {abs(phase_8):.1f}° {'✓' if abs(phase_8) < 30 else '✗'} (要求<30°)")
print(f"  8Hz幅值变化: {abs(mag_8-1)*100:.1f}% {'✓' if abs(mag_8-1) < 0.2 else '✗'} (要求<20%)")

# 绘制频率响应
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

# 幅频响应
ax1 = axes[0]
ax1.plot(freqs, mag_lp_db, 'g--', linewidth=1.5, label=f'Lowpass ({fc_lp}Hz)', alpha=0.7)
ax1.plot(freqs, mag_notch_db, 'r:', linewidth=1.5, label=f'Notch ({f0_notch}Hz)', alpha=0.7)
ax1.plot(freqs, mag_cascaded_db, 'b-', linewidth=2, label='Cascaded')
ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
ax1.axhline(y=-14, color='r', linestyle='--', alpha=0.5, label='-14dB (80% attenuation)')
ax1.set_xlabel('Frequency (Hz)')
ax1.set_ylabel('Magnitude (dB)')
ax1.set_title('Cascaded Filter Frequency Response')
ax1.legend(loc='lower left', fontsize=8)
ax1.grid(True, alpha=0.3)
ax1.set_xlim([0, 25])
ax1.set_ylim([-40, 5])

# 相频响应
ax2 = axes[1]
ax2.plot(freqs, phase_cascaded_deg, 'b-', linewidth=2)
ax2.axhline(y=30, color='gray', linestyle='--', alpha=0.5, label='±30°')
ax2.axhline(y=-30, color='gray', linestyle='--', alpha=0.5)
ax2.set_xlabel('Frequency (Hz)')
ax2.set_ylabel('Phase (degrees)')
ax2.set_title('Cascaded Filter Phase Response')
ax2.legend(loc='lower left')
ax2.grid(True, alpha=0.3)
ax2.set_xlim([0, 25])
ax2.set_ylim([-90, 90])

plt.tight_layout()
plt.savefig('cascaded_filter_design.png', dpi=150, bbox_inches='tight')
print(f"\n滤波器设计图已保存: cascaded_filter_design.png")

# 输出C语言代码
print(f"\n{'='*80}")
print("C语言实现代码 - 级联滤波器")
print(f"{'='*80}")
print(f"""
/* 级联滤波器参数
 * 第一级：一阶低通滤波器，截止频率{fc_lp}Hz
 * 第二级：二阶陷波滤波器，中心频率{f0_notch}Hz
 */

/* 低通滤波器参数 */
#define LPF_ALPHA    {alpha_lp:.6f}f

/* 陷波滤波器参数 */
#define NOTCH_B0    {b_notch[0]:.6f}f
#define NOTCH_B1    {b_notch[1]:.6f}f
#define NOTCH_B2    {b_notch[2]:.6f}f
#define NOTCH_A1    {a_notch[1]:.6f}f
#define NOTCH_A2    {a_notch[2]:.6f}f

/* 滤波器状态 */
static float s_lpf_state = 0.0f;
static float s_notch_x1 = 0.0f, s_notch_x2 = 0.0f;
static float s_notch_y1 = 0.0f, s_notch_y2 = 0.0f;
static int s_filter_initialized = 0;

float apply_cascaded_filter(float x0) {{
    if (!s_filter_initialized) {{
        s_lpf_state = x0;
        s_notch_x1 = s_notch_x2 = x0;
        s_notch_y1 = s_notch_y2 = x0;
        s_filter_initialized = 1;
    }}
    
    /* 第一级：低通滤波 */
    float y_lp = LPF_ALPHA * x0 + (1.0f - LPF_ALPHA) * s_lpf_state;
    s_lpf_state = y_lp;
    
    /* 第二级：陷波滤波 */
    float y_notch = NOTCH_B0 * y_lp + NOTCH_B1 * s_notch_x1 + NOTCH_B2 * s_notch_x2
                    - NOTCH_A1 * s_notch_y1 - NOTCH_A2 * s_notch_y2;
    
    s_notch_x2 = s_notch_x1;
    s_notch_x1 = y_lp;
    s_notch_y2 = s_notch_y1;
    s_notch_y1 = y_notch;
    
    return y_notch;
}}

void reset_cascaded_filter(void) {{
    s_filter_initialized = 0;
    s_lpf_state = 0.0f;
    s_notch_x1 = s_notch_x2 = 0.0f;
    s_notch_y1 = s_notch_y2 = 0.0f;
}}
""")
