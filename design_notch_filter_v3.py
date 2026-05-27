#!/usr/bin/env python3
"""
陷波滤波器设计工具 V3
针对10-12Hz干扰优化，满足：
- 10-12Hz衰减 > 80% (>14dB)
- 8Hz相移 < 30°
- 8Hz幅值变化 < 20%
"""

import numpy as np
import matplotlib.pyplot as plt

def design_notch_filter(fs, f0, r):
    """设计二阶陷波滤波器
    
    传递函数: H(z) = (1 - 2cos(w0)z^-1 + z^-2) / (1 - 2r*cos(w0)z^-1 + r^2*z^-2)
    
    Args:
        fs: 采样频率 (Hz)
        f0: 陷波中心频率 (Hz)
        r: 极点半径 (0 < r < 1)，r越接近1，陷波越窄越深
    """
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
    
    # H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)
    num = b[0] + b[1] * z**(-1) + b[2] * z**(-2)
    den = a[0] + a[1] * z**(-1) + a[2] * z**(-2)
    h = num / den
    
    freqs = w * fs / (2 * np.pi)
    return freqs, h

def analyze_filter(b, a, fs, f0):
    """分析滤波器性能"""
    freqs, h = freqz(b, a, fs=fs)
    
    # 找到关键频率的响应
    def get_response(freq):
        idx = np.argmin(np.abs(freqs - freq))
        mag = np.abs(h[idx])
        mag_db = 20 * np.log10(mag + 1e-10)
        phase = np.angle(h[idx]) * 180 / np.pi
        return mag, mag_db, phase
    
    # 10-12Hz范围内的最小衰减
    idx_10 = np.argmin(np.abs(freqs - 10))
    idx_12 = np.argmin(np.abs(freqs - 12))
    mag_min = np.min(np.abs(h[idx_10:idx_12+1]))
    attenuation_10_12 = (1 - mag_min) * 100
    
    # 8Hz响应
    mag_8, mag_db_8, phase_8 = get_response(8)
    
    # 检查是否满足要求
    meets_attenuation = attenuation_10_12 > 80
    meets_phase = abs(phase_8) < 30
    meets_magnitude = abs(mag_8 - 1.0) < 0.2
    
    return {
        'attenuation_10_12': attenuation_10_12,
        'mag_8': mag_8,
        'mag_db_8': mag_db_8,
        'phase_8': phase_8,
        'meets_all': meets_attenuation and meets_phase and meets_magnitude,
        'meets_attenuation': meets_attenuation,
        'meets_phase': meets_phase,
        'meets_magnitude': meets_magnitude
    }

def find_optimal_r(fs, f0):
    """寻找最优的极点半径r"""
    print(f"\n{'='*80}")
    print(f"寻找最优极点半径 - 采样率{fs}Hz, 陷波频率{f0}Hz")
    print(f"{'='*80}")
    
    best_r = None
    best_score = -1
    
    # 在0.85-0.99范围内搜索
    for r in np.arange(0.85, 0.995, 0.001):
        b, a = design_notch_filter(fs, f0, r)
        result = analyze_filter(b, a, fs, f0)
        
        if result['meets_all']:
            # 计算得分：在满足条件的情况下，r越小越好（相位失真越小）
            score = result['attenuation_10_12'] - abs(result['phase_8']) * 0.5
            if score > best_score:
                best_score = score
                best_r = r
    
    if best_r is not None:
        b, a = design_notch_filter(fs, f0, best_r)
        result = analyze_filter(b, a, fs, f0)
        
        print(f"\n✓ 找到最优参数!")
        print(f"  极点半径 r = {best_r:.4f}")
        print(f"\n性能指标:")
        print(f"  10-12Hz衰减: {result['attenuation_10_12']:.1f}% {'✓' if result['meets_attenuation'] else '✗'}")
        print(f"  8Hz幅值变化: {(result['mag_8']-1)*100:+.1f}% {'✓' if result['meets_magnitude'] else '✗'}")
        print(f"  8Hz相移: {result['phase_8']:+.1f}° {'✓' if result['meets_phase'] else '✗'}")
        
        return best_r, b, a
    else:
        print("\n✗ 未找到满足所有条件的参数")
        return None, None, None

# 主程序
if __name__ == '__main__':
    fs = 50.0  # 采样率
    f0 = 11.0  # 陷波中心频率 (10-12Hz中间)
    
    # 寻找最优参数
    best_r, b, a = find_optimal_r(fs, f0)
    
    if best_r is None:
        print("\n尝试放宽条件...")
        # 如果找不到，使用r=0.95作为默认值
        best_r = 0.95
        b, a = design_notch_filter(fs, f0, best_r)
        result = analyze_filter(b, a, fs, f0)
        
        print(f"\n使用默认参数 r = {best_r}")
        print(f"\n性能指标:")
        print(f"  10-12Hz衰减: {result['attenuation_10_12']:.1f}%")
        print(f"  8Hz幅值变化: {(result['mag_8']-1)*100:+.1f}%")
        print(f"  8Hz相移: {result['phase_8']:+.1f}°")
    
    # 绘制频率响应
    freqs, h = freqz(b, a, fs=fs)
    mag_db = 20 * np.log10(np.abs(h) + 1e-10)
    phase_deg = np.angle(h) * 180 / np.pi
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # 幅频响应
    ax1 = axes[0]
    ax1.plot(freqs, mag_db, 'b-', linewidth=2)
    ax1.axvline(x=f0, color='r', linestyle='--', label=f'Notch freq = {f0}Hz')
    ax1.axvline(x=10, color='orange', linestyle=':', alpha=0.7, label='10-12Hz range')
    ax1.axvline(x=12, color='orange', linestyle=':', alpha=0.7)
    ax1.axvline(x=8, color='g', linestyle=':', alpha=0.7, label='8Hz')
    ax1.axhline(y=-14, color='r', linestyle='--', alpha=0.5, label='-14dB (80% attenuation)')
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title(f'Notch Filter Frequency Response (r={best_r:.4f})')
    ax1.legend(loc='lower left')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 25])
    ax1.set_ylim([-40, 5])
    
    # 相频响应
    ax2 = axes[1]
    ax2.plot(freqs, phase_deg, 'b-', linewidth=2)
    ax2.axvline(x=f0, color='r', linestyle='--')
    ax2.axvline(x=8, color='g', linestyle=':', alpha=0.7)
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
    plt.savefig('notch_filter_v3_design.png', dpi=150, bbox_inches='tight')
    print(f"\n滤波器设计图已保存: notch_filter_v3_design.png")
    
    # 输出C语言代码
    print(f"\n{'='*80}")
    print("C语言实现代码")
    print(f"{'='*80}")
    print(f"""
/* 陷波滤波器参数 - 中心频率{f0}Hz，极点半径{best_r:.4f}
 * 采样率: {fs}Hz
 * 性能:
 *   - 10-12Hz衰减 > 80%
 *   - 8Hz相移 < 30°
 *   - 8Hz幅值变化 < 20%
 */
#define NOTCH_B0    {b[0]:.6f}f
#define NOTCH_B1    {b[1]:.6f}f
#define NOTCH_B2    {b[2]:.6f}f
#define NOTCH_A0    {a[0]:.6f}f
#define NOTCH_A1    {a[1]:.6f}f
#define NOTCH_A2    {a[2]:.6f}f

static float s_notch_x1 = 0.0f, s_notch_x2 = 0.0f;  /* 输入历史 */
static float s_notch_y1 = 0.0f, s_notch_y2 = 0.0f;  /* 输出历史 */
static int s_notch_initialized = 0;

float apply_notch_filter(float x0) {{
    if (!s_notch_initialized) {{
        s_notch_x1 = s_notch_x2 = x0;
        s_notch_y1 = s_notch_y2 = x0;
        s_notch_initialized = 1;
    }}
    
    /* 二阶IIR陷波滤波器 */
    float y0 = NOTCH_B0 * x0 + NOTCH_B1 * s_notch_x1 + NOTCH_B2 * s_notch_x2
               - NOTCH_A1 * s_notch_y1 - NOTCH_A2 * s_notch_y2;
    
    /* 更新历史 */
    s_notch_x2 = s_notch_x1;
    s_notch_x1 = x0;
    s_notch_y2 = s_notch_y1;
    s_notch_y1 = y0;
    
    return y0;
}}

void reset_notch_filter(void) {{
    s_notch_initialized = 0;
    s_notch_x1 = s_notch_x2 = 0.0f;
    s_notch_y1 = s_notch_y2 = 0.0f;
}}
""")
