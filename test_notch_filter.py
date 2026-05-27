#!/usr/bin/env python3
"""
测试陷波滤波器对三个不同频率信号的响应
信号1：5Hz，幅值1
信号2：8Hz，幅值1  
信号3：11Hz，幅值1
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# 滤波器参数（从设计结果）
fs = 50.0  # 采样率
f0 = 11.0  # 陷波中心频率
Q = 1.30   # 品质因数

# 设计陷波滤波器
b, a = signal.iirnotch(f0, Q, fs)

# 生成测试信号
duration = 2.0  # 2秒
t = np.linspace(0, duration, int(fs * duration), endpoint=False)

# 三个测试信号
freq1, freq2, freq3 = 5.0, 8.0, 11.0
signal1 = np.sin(2 * np.pi * freq1 * t)
signal2 = np.sin(2 * np.pi * freq2 * t)
signal3 = np.sin(2 * np.pi * freq3 * t)

# 通过滤波器
output1 = signal.lfilter(b, a, signal1)
output2 = signal.lfilter(b, a, signal2)
output3 = signal.lfilter(b, a, signal3)

# 等待瞬态响应稳定后计算衰减（取后1秒）
stable_start = int(fs * 1.0)  # 1秒后开始

def calculate_attenuation(input_sig, output_sig, stable_start):
    """计算稳定后的幅值衰减"""
    input_amp = np.max(np.abs(input_sig[stable_start:]))
    output_amp = np.max(np.abs(output_sig[stable_start:]))
    attenuation = (1 - output_amp / input_amp) * 100
    return input_amp, output_amp, attenuation

amp1_in, amp1_out, att1 = calculate_attenuation(signal1, output1, stable_start)
amp2_in, amp2_out, att2 = calculate_attenuation(signal2, output2, stable_start)
amp3_in, amp3_out, att3 = calculate_attenuation(signal3, output3, stable_start)

print("=" * 80)
print("陷波滤波器测试 - 三个频率信号")
print("=" * 80)
print(f"滤波器参数: Q={Q}, f0={f0}Hz, fs={fs}Hz")
print(f"\n信号1 ({freq1}Hz): 输入幅值={amp1_in:.4f}, 输出幅值={amp1_out:.4f}, 衰减={att1:.1f}%")
print(f"信号2 ({freq2}Hz): 输入幅值={amp2_in:.4f}, 输出幅值={amp2_out:.4f}, 衰减={att2:.1f}%")
print(f"信号3 ({freq3}Hz): 输入幅值={amp3_in:.4f}, 输出幅值={amp3_out:.4f}, 衰减={att3:.1f}%")

# 绘制结果
fig, axes = plt.subplots(3, 1, figsize=(14, 10))
fig.suptitle(f'Notch Filter Test (Q={Q}, f0={f0}Hz)', fontsize=16, fontweight='bold')

colors = ['blue', 'green', 'red']
freqs = [freq1, freq2, freq3]
signals = [signal1, signal2, signal3]
outputs = [output1, output2, output3]
attenuations = [att1, att2, att3]

for i, (ax, freq, sig, out, att, color) in enumerate(zip(axes, freqs, signals, outputs, attenuations, colors)):
    # 绘制输入信号
    ax.plot(t, sig, color=color, linewidth=1.5, alpha=0.7, 
            label=f'Input {freq}Hz (Amp=1.0)', linestyle='--')
    
    # 绘制输出信号
    ax.plot(t, out, color=color, linewidth=2, 
            label=f'Output {freq}Hz (Amp={np.max(np.abs(out[stable_start:])):.3f}, Att={att:.1f}%)')
    
    # 添加稳态区域标记
    ax.axvline(x=1.0, color='gray', linestyle=':', alpha=0.5)
    ax.axvspan(1.0, 2.0, alpha=0.1, color='yellow', label='Stable region')
    
    ax.set_ylabel('Amplitude', fontsize=11)
    ax.set_title(f'{freq}Hz Signal - Attenuation: {att:.1f}%', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 2])
    ax.set_ylim([-1.5, 1.5])
    
    # 只在最后一个子图添加x轴标签
    if i == 2:
        ax.set_xlabel('Time (s)', fontsize=11)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('notch_filter_test_3signals.png', dpi=150, bbox_inches='tight')
print(f"\n测试图已保存: notch_filter_test_3signals.png")

# 计算理论值对比
print(f"\n{'='*80}")
print("理论vs实际对比")
print(f"{'='*80}")
w, h = signal.freqz(b, a, worN=8192, fs=fs)

def get_theoretical_attenuation(freq, w, h):
    idx = np.argmin(np.abs(w - freq))
    mag = np.abs(h[idx])
    return (1 - mag) * 100

for freq, actual_att in zip([freq1, freq2, freq3], [att1, att2, att3]):
    theoretical_att = get_theoretical_attenuation(freq, w, h)
    print(f"{freq}Hz: 理论衰减={theoretical_att:.1f}%, 实际衰减={actual_att:.1f}%, 误差={abs(theoretical_att-actual_att):.1f}%")
