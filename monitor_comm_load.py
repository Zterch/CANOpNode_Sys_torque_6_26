#!/usr/bin/env python3
"""
通信负载监控脚本 - 实时显示CAN总线和串口通信负载
用法：
    python3 monitor_comm_load.py
或后台运行：
    python3 monitor_comm_load.py > comm_load.log 2>&1 &
"""

import os
import re
import sys
import time
import subprocess

CAN_INTERFACE = "can0"
SERIAL_DEVICES = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]


def get_can_stats(iface):
    """获取CAN接口统计信息"""
    try:
        output = subprocess.check_output(
            ["ip", "-s", "-d", "link", "show", iface],
            stderr=subprocess.DEVNULL, text=True
        )
    except Exception as e:
        return None, f"无法读取CAN统计: {e}"

    stats = {"rx_packets": 0, "tx_packets": 0, "rx_bytes": 0, "tx_bytes": 0,
             "rx_errors": 0, "tx_errors": 0, "rx_dropped": 0, "tx_dropped": 0,
             "overruns": 0}

    for line in output.splitlines():
        line = line.strip()
        # RX packets: 12345 bytes: 67890 errors: 0 dropped: 0 overrun: 0 mcast: 0
        m = re.match(r"RX packets:\s+(\d+)\s+bytes:\s+(\d+)\s+errors:\s+(\d+)\s+dropped:\s+(\d+)\s+overrun:\s+(\d+)", line)
        if m:
            stats["rx_packets"] = int(m.group(1))
            stats["rx_bytes"] = int(m.group(2))
            stats["rx_errors"] = int(m.group(3))
            stats["rx_dropped"] = int(m.group(4))
            stats["overruns"] += int(m.group(5))
            continue
        # TX packets: 12345 bytes: 67890 errors: 0 dropped: 0 carrier: 0 collsns: 0
        m = re.match(r"TX packets:\s+(\d+)\s+bytes:\s+(\d+)\s+errors:\s+(\d+)\s+dropped:\s+(\d+)", line)
        if m:
            stats["tx_packets"] = int(m.group(1))
            stats["tx_bytes"] = int(m.group(2))
            stats["tx_errors"] = int(m.group(3))
            stats["tx_dropped"] = int(m.group(4))
            continue
        # 旧格式:     RX: bytes packets errors dropped overrun mcast
        m = re.match(r"RX:\s+bytes:\s+(\d+)\s+packets:\s+(\d+)\s+errors:\s+(\d+)\s+dropped:\s+(\d+)\s+overrun:\s+(\d+)", line)
        if m:
            stats["rx_bytes"] = int(m.group(1))
            stats["rx_packets"] = int(m.group(2))
            stats["rx_errors"] = int(m.group(3))
            stats["rx_dropped"] = int(m.group(4))
            stats["overruns"] += int(m.group(5))
            continue
        m = re.match(r"TX:\s+bytes:\s+(\d+)\s+packets:\s+(\d+)\s+errors:\s+(\d+)\s+dropped:\s+(\d+)", line)
        if m:
            stats["tx_bytes"] = int(m.group(1))
            stats["tx_packets"] = int(m.group(2))
            stats["tx_errors"] = int(m.group(3))
            stats["tx_dropped"] = int(m.group(4))
            continue

    return stats, None


def get_serial_speed(device):
    """获取串口波特率"""
    try:
        output = subprocess.check_output(["stty", "-F", device], stderr=subprocess.DEVNULL, text=True)
        m = re.search(r"speed\s+(\d+)", output)
        if m:
            return int(m.group(1))
    except Exception:
        pass
    return None


def count_tty_rx_tx(device):
    """
    尝试从 /sys 或 /proc 获取串口统计。
    Linux 串口驱动通常不导出标准统计，这里尽量获取。
    """
    stats = {"rx": 0, "tx": 0}
    # /sys/class/tty/ttyUSB0/device 下的统计因设备而异，不通用
    # 使用 lsof 看是否有进程打开
    try:
        subprocess.check_output(["lsof", device], stderr=subprocess.DEVNULL, text=True)
        stats["open"] = True
    except Exception:
        stats["open"] = False
    return stats


def format_can_line(label, val, last):
    if last is None:
        return f"{label}: {val} (initial)"
    delta = val - last
    return f"{label}: {val} (+{delta}/s)"


def main():
    print("=" * 70)
    print("通信负载监控 - 1Hz 刷新")
    print("CAN接口:", CAN_INTERFACE)
    print("串口设备:", ", ".join(SERIAL_DEVICES))
    print("=" * 70)

    last_can_stats = None
    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        can_stats, err = get_can_stats(CAN_INTERFACE)
        if err:
            print(f"[CAN] {err}")
        else:
            print(f"\n[{time.strftime('%H:%M:%S')}] CAN {CAN_INTERFACE}")
            print(f"  {format_can_line('RX packets', can_stats['rx_packets'], last_can_stats['rx_packets'] if last_can_stats else None)}")
            print(f"  {format_can_line('TX packets', can_stats['tx_packets'], last_can_stats['tx_packets'] if last_can_stats else None)}")
            print(f"  {format_can_line('RX bytes ', can_stats['rx_bytes'], last_can_stats['rx_bytes'] if last_can_stats else None)}")
            print(f"  {format_can_line('TX bytes ', can_stats['tx_bytes'], last_can_stats['tx_bytes'] if last_can_stats else None)}")
            print(f"  RX errors: {can_stats['rx_errors']}  TX errors: {can_stats['tx_errors']}  dropped: {can_stats['rx_dropped'] + can_stats['tx_dropped']}  overruns: {can_stats['overruns']}")
            if last_can_stats and dt > 0:
                rx_pps = (can_stats["rx_packets"] - last_can_stats["rx_packets"]) / dt
                tx_pps = (can_stats["tx_packets"] - last_can_stats["tx_packets"]) / dt
                rx_bps = (can_stats["rx_bytes"] - last_can_stats["rx_bytes"]) / dt * 8
                tx_bps = (can_stats["tx_bytes"] - last_can_stats["tx_bytes"]) / dt * 8
                print(f"  RX/TX pps: {rx_pps:.1f}/{tx_pps:.1f},  RX/TX bps: {rx_bps:.0f}/{tx_bps:.0f}")
                print(f"  CAN utilization: {(rx_bps + tx_bps) / 1e6:.2f}% @ 1Mbps")

        last_can_stats = can_stats

        print("  Serial ports:")
        for dev in SERIAL_DEVICES:
            if not os.path.exists(dev):
                print(f"    {dev}: not present")
                continue
            speed = get_serial_speed(dev)
            open_info = count_tty_rx_tx(dev)
            speed_str = f"{speed} bps" if speed else "speed unknown"
            print(f"    {dev}: {speed_str}, open={open_info.get('open', False)}")

        print("-" * 70)
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n监控停止")
        sys.exit(0)
