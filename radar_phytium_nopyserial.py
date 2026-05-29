#!/usr/bin/env python3
"""
MS60-1211S80M-BSD (AT6010) 雷达调试工具 — 飞腾派 PE2204 版
纯 Python 实现，不依赖 pyserial，使用 Linux termios

飞腾派串口映射:
  - J1 DEBUG_UART1 (Pin7=TXD, Pin9=RXD) → /dev/ttyAMA1 (0x2800D000) [console占用]
  - 40pin/J1 UART2 (Pin8=TXD, Pin10=RXD) → /dev/ttyAMA2 (0x2800E000)

用法:
  python3 radar_phytium_nopyserial.py                  默认: 配置BSD + 监控
  python3 radar_phytium_nopyserial.py -m               仅监控 (不发送配置命令)
  python3 radar_phytium_nopyserial.py -s               波特率扫描
  python3 radar_phytium_nopyserial.py -p /dev/ttyAMA2  指定串口
  python3 radar_phytium_nopyserial.py 115200           指定波特率
"""

import os
import sys
import time
import struct
import csv
import fcntl
import termios
import select
from datetime import datetime

PORT = "/dev/ttyAMA2"
BAUD = 921600

HEAD_REPORT = 0x5A
HEAD_REPLY  = 0x59
HEAD_CMD    = 0x58
TYPE_BSD    = 7

BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}

TYPE_NAMES = {
    0: "FMCW完整", 1: "HTM高度", 2: "电平探测",
    3: "运动存在", 4: "呼吸心率", 5: "分区检测",
    6: "简化BHR", 7: "BSD盲区",
}

ALARM_WARN_DIST = 15
ALARM_WARN_SPEED = -2
ALARM_DANGER_DIST = 8
ALARM_DANGER_SPEED = -4
ALARM_CRITICAL_DIST = 4
ALARM_CRITICAL_SPEED = -1

ALARM_LEVELS = [
    ("CRITICAL", ALARM_CRITICAL_DIST, ALARM_CRITICAL_SPEED, "\033[91m"),
    ("DANGER", ALARM_DANGER_DIST, ALARM_DANGER_SPEED, "\033[93m"),
    ("WARN", ALARM_WARN_DIST, ALARM_WARN_SPEED, "\033[96m"),
]


class SerialPort:
    def __init__(self, port, baudrate=921600, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.fd = -1

    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if self.fd < 0:
            raise OSError(f"无法打开 {self.port}")

        attrs = termios.tcgetattr(self.fd)
        baud = BAUD_MAP.get(self.baudrate, termios.B921600)

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | baud
        attrs[3] = 0
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = int(self.timeout * 10)

        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        return self

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def write(self, data):
        return os.write(self.fd, data)

    def read(self, size=128):
        ready, _, _ = select.select([self.fd], [], [], self.timeout)
        if ready:
            try:
                return os.read(self.fd, size)
            except OSError:
                return b""
        return b""

    def in_waiting(self):
        buf = bytearray(4)
        try:
            fcntl.ioctl(self.fd, termios.FIONREAD, buf)
            return struct.unpack("I", buf)[0]
        except OSError:
            return 0

    def flush(self):
        termios.tcdrain(self.fd)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()


def calc_checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def calc_checksum16(data: bytes) -> bytes:
    s = sum(data)
    return bytes([s & 0xFF, (s >> 8) & 0xFF])


def build_cmd(group: int, cmd: int, params: bytes = b"") -> bytes:
    cmd_byte = (group << 5) | cmd
    frame = bytes([HEAD_CMD, cmd_byte, len(params)]) + params
    return frame + calc_checksum16(frame)


def send_cmd(ser: SerialPort, group: int, cmd: int, params: bytes = b"",
             name: str = "") -> bytes:
    frame = build_cmd(group, cmd, params)
    hex_str = " ".join(f"{b:02X}" for b in frame)
    label = f"  发送: {name} [{hex_str}]"
    print(label.ljust(100))
    ser.write(frame)
    ser.flush()
    time.sleep(0.3)

    reply = b""
    t0 = time.time()
    while time.time() - t0 < 0.5:
        chunk = ser.read(128)
        if chunk:
            reply += chunk
        else:
            time.sleep(0.02)

    if reply:
        ok = len(reply) >= 5 and calc_checksum16(reply[:-2]) == reply[-2:]
        status = "OK" if ok else ("CK16_ERR" if len(reply) >= 5 else "SHORT")
        hex_reply = " ".join(f"{b:02X}" for b in reply)
        if len(reply) >= 4:
            rb = reply[1]
            grp = (rb >> 5)
            c = rb & 0x1F
            plen = reply[2]
            expected = 5 + plen
            if len(reply) < expected:
                status += f" NEED={expected}"
            print(f"  [回复] G{grp}.0x{c:02X} Len={plen} {status} | {hex_reply}")
        else:
            print(f"  回复: {hex_reply}")
    else:
        print(f"  [回复] 无响应")
    return reply


def read_version(ser: SerialPort):
    send_cmd(ser, 7, 0x1E, name="获取版本(0xFE)")


def read_status(ser: SerialPort):
    send_cmd(ser, 1, 0x01, name="获取状态(0x21)")


def enable_bsd(ser: SerialPort):
    send_cmd(ser, 6, 0x11, bytes([0x01]), name="开启感测功能(0xD1)")


def enable_bsd_detection(ser: SerialPort):
    send_cmd(ser, 6, 0x10, bytes([0x01]), name="开启BSD检测(0xD0)")


def enable_auto_report(ser: SerialPort):
    send_cmd(ser, 6, 0x12, bytes([0x01]), name="开启自动上报(0xD2)")


def set_baud(ser: SerialPort, baud: int):
    idx_map = {9600:0, 19200:1, 38400:2, 57600:3, 115200:4, 256000:5, 460800:6, 921600:7}
    idx = idx_map.get(baud, 7)
    send_cmd(ser, 0, 0x19, bytes([idx]), name=f"切换波特率→{baud}")


def set_sense_time(ser: SerialPort, ms: int = 100):
    send_cmd(ser, 0, 0x04, struct.pack("<I", ms), name=f"电平保持时间={ms}ms")


def check_alarm(dist, speed):
    for level, d_thresh, s_thresh, color in ALARM_LEVELS:
        if dist <= d_thresh and speed <= s_thresh:
            return level, color
    return None, ""


def parse_bsd_targets(raw: bytes):
    if raw[0] != HEAD_REPORT or len(raw) < 4:
        return None
    msg_type = raw[2]
    if msg_type != TYPE_BSD:
        return None
    ok = (calc_checksum8(raw[:-1]) == raw[-1])
    if not ok:
        return None
    payload = raw[3:-1]
    if len(payload) < 4:
        return None
    obj_num = struct.unpack_from("<H", payload, 0)[0]
    obj_num = min(obj_num, 8)
    targets = []
    for i in range(obj_num):
        off = 4 + i * 4
        if off + 4 > len(payload):
            break
        rng, ang, vel, oid = struct.unpack_from("<bbbb", payload, off)
        targets.append({"id": oid, "dist": rng, "angle": ang, "speed": vel})
    return targets


class DataLogger:
    def __init__(self, filename=None):
        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"radar_log_{ts}.csv"
        self.filename = filename
        self.f = open(filename, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.f)
        self.writer.writerow(["timestamp", "elapsed_s", "obj_id", "dist_m",
                              "angle_deg", "speed_ms", "alarm"])
        self.f.flush()
        self.count = 0

    def log(self, elapsed, targets):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        for t in targets:
            alarm, _ = check_alarm(t["dist"], t["speed"])
            alarm_str = alarm or ""
            self.writer.writerow([ts, f"{elapsed:.3f}", t["id"], t["dist"],
                                  t["angle"], t["speed"], alarm_str])
            self.count += 1
        self.f.flush()

    def close(self):
        self.f.close()
        return self.filename, self.count


def parse_frame(raw: bytes, logger=None, elapsed=0):
    head = raw[0]
    if head == HEAD_REPORT:
        msg_type = raw[2] if len(raw) > 2 else -1
        ok = (calc_checksum8(raw[:-1]) == raw[-1])
        status = "OK" if ok else "CK_ERR"
        tname = TYPE_NAMES.get(msg_type, f"T={msg_type}")
        if msg_type == TYPE_BSD and ok:
            targets = parse_bsd_targets(raw)
            if targets is not None:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{ts}] BSD | {len(targets)}目标 | {status}")
                for t in targets:
                    alarm, color = check_alarm(t["dist"], t["speed"])
                    alarm_str = f" {color}<<< {alarm} >>>\033[0m" if alarm else ""
                    print(f"  ID{t['id']:+d} 距离{t['dist']:+d}m "
                          f"角度{t['angle']:+d}° 速度{t['speed']:+d}m/s{alarm_str}")
                if logger:
                    logger.log(elapsed, targets)
                return
        hex_str = " ".join(f"{b:02X}" for b in raw)
        print(f"  [{tname}] {status} | {hex_str}")
    elif head == HEAD_REPLY:
        ok = len(raw) >= 5 and calc_checksum16(raw[:-2]) == raw[-2:]
        status = "OK" if ok else ("CK_ERR" if len(raw) >= 5 else "SHORT")
        cmd_byte = raw[1] if len(raw) > 1 else 0
        param_len = raw[2] if len(raw) > 2 else 0
        hex_str = " ".join(f"{b:02X}" for b in raw)
        grp = cmd_byte >> 5
        c = cmd_byte & 0x1F
        print(f"  [回复] G{grp}.0x{c:02X} Len={param_len} {status} | {hex_str}")
    elif head == HEAD_CMD:
        hex_str = " ".join(f"{b:02X}" for b in raw)
        print(f"  [命令] {hex_str}")
    else:
        hex_str = " ".join(f"{b:02X}" for b in raw)
        print(f"  [RAW] {hex_str}")


def parse_stream(buf: bytearray, logger=None, elapsed=0):
    while True:
        idx5a = buf.find(b'\x5A')
        idx59 = buf.find(b'\x59')
        idx58 = buf.find(b'\x58')

        candidates = []
        if idx5a >= 0: candidates.append(idx5a)
        if idx59 >= 0: candidates.append(idx59)
        if idx58 >= 0: candidates.append(idx58)
        if not candidates:
            break

        idx = min(candidates)
        if idx > 0:
            del buf[:idx]

        if len(buf) < 3:
            break

        head = buf[0]
        if head == HEAD_REPORT:
            length_byte = buf[1]
            frame_total = 3 + length_byte
        elif head in (HEAD_REPLY, HEAD_CMD):
            param_len = buf[2]
            frame_total = 5 + param_len
        else:
            del buf[:1]
            continue

        if frame_total < 3 or len(buf) < frame_total:
            break

        frame_raw = bytes(buf[:frame_total])
        parse_frame(frame_raw, logger, elapsed)
        del buf[:frame_total]


def open_port(port, baud):
    try:
        ser = SerialPort(port, baud)
        ser.open()
        return ser
    except OSError as e:
        print(f"[ERROR] {e}")
        return None


def print_alarm_config():
    print("  报警阈值配置:")
    for level, d, s, _ in ALARM_LEVELS:
        print(f"    {level:8s}: 距离 ≤ {d:2d}m  且  速度 ≤ {s:+d}m/s (接近)")
    print()


def monitor(port: str, baud: int, do_config: bool = True, log_file: str = None):
    print()
    print("=" * 70)
    print(f"  MS60-1211S80M-BSD | Port: {port} | Baud: {baud}")
    print(f"  飞腾派 PE2204 — UART2 (J1 Pin8/Pin10)")
    print("=" * 70)
    print_alarm_config()

    ser = open_port(port, baud)
    if ser is None:
        return
    print(f"[OK] 串口已打开\n")

    if do_config:
        print("─── 发送配置命令 ───")
        read_version(ser)
        read_status(ser)
        enable_bsd(ser)
        enable_bsd_detection(ser)
        enable_auto_report(ser)
        print("─── 配置完成，等待数据 ───\n")

    logger = DataLogger(log_file)
    print(f"[LOG] 数据记录到: {logger.filename}\n")

    buf = bytearray()
    total = 0
    t0 = time.time()

    try:
        while True:
            elapsed = time.time() - t0
            chunk = ser.read(128)
            if chunk:
                buf.extend(chunk)
                total += len(chunk)
                parse_stream(buf, logger, elapsed)
            else:
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        fname, count = logger.close()
        ser.close()
        print(f"\n[DONE] 总计 {total}B | 运行 {time.time()-t0:.0f}s | "
              f"记录 {count} 条 → {fname}")


def baud_scan(port: str):
    rates = [9600, 19200, 38400, 57600, 115200, 256000, 460800, 921600]
    print(f"\n波特率扫描: {port}")
    print("=" * 50)

    for baud in rates:
        print(f"\n--- {baud} ---")
        ser = open_port(port, baud)
        if ser is None:
            continue
        t0 = time.time()
        buf = bytearray()
        got = False
        while time.time() - t0 < 2.0:
            chunk = ser.read(128)
            if chunk:
                buf.extend(chunk)
                got = True
            else:
                time.sleep(0.05)
        ser.close()
        if got:
            print(f"  [HIT] {len(buf)}B @ {baud}")
            print(f"  {' '.join(f'{b:02X}' for b in buf[:64])}")
            return baud
        else:
            print(f"  [--] 无数据")
    print(f"\n未找到有效波特率")
    return None


def print_usage():
    print("用法:")
    print("  python3 radar_phytium_nopyserial.py                    默认: 配置BSD + 监控 + CSV记录")
    print("  python3 radar_phytium_nopyserial.py -m                 仅监控 (不发送配置命令)")
    print("  python3 radar_phytium_nopyserial.py -s                 波特率扫描")
    print("  python3 radar_phytium_nopyserial.py -p /dev/ttyAMA2    指定串口设备")
    print("  python3 radar_phytium_nopyserial.py -l FILE            指定CSV日志文件名")
    print("  python3 radar_phytium_nopyserial.py 115200             指定波特率")
    print()
    print("飞腾派串口映射:")
    print("  /dev/ttyAMA1  ← J1 DEBUG_UART1 (Pin7=TXD, Pin9=RXD) [console占用]")
    print("  /dev/ttyAMA2  ← UART2 (Pin8=TXD, Pin10=RXD)")
    print()
    print("报警阈值 (修改脚本顶部常量):")
    for level, d, s, _ in ALARM_LEVELS:
        print(f"  {level:8s}: 距离 ≤ {d}m  且  速度 ≤ {s}m/s")


if __name__ == "__main__":
    print(f"╔══════════════════════════════════════════════════════╗")
    print(f"║  Radar Debug — Phytium PE2204  |  {PORT} @ {BAUD}  ║")
    print(f"╚══════════════════════════════════════════════════════╝")

    args = sys.argv[1:]
    do_config = True
    scan_mode = False
    monitor_only = False
    log_file = None
    custom_baud = None
    custom_port = None

    i = 0
    while i < len(args):
        a = args[i]
        if a in ("-m", "--monitor"):
            monitor_only = True
        elif a in ("-s", "--scan"):
            scan_mode = True
        elif a in ("-l", "--log"):
            i += 1
            if i < len(args):
                log_file = args[i]
        elif a in ("-p", "--port"):
            i += 1
            if i < len(args):
                custom_port = args[i]
        elif a in ("-h", "--help"):
            print_usage()
            sys.exit(0)
        elif a.isdigit():
            custom_baud = int(a)
        i += 1

    port = custom_port or PORT
    baud = custom_baud or BAUD

    if scan_mode:
        found = baud_scan(port)
        if found:
            monitor(port, found, log_file=log_file)
    else:
        monitor(port, baud, do_config=not monitor_only, log_file=log_file)
