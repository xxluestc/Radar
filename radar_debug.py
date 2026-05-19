"""
MS60-1211S80M-BSD (AT6010) 雷达USB转TTL调试工具 v3
支持:
  - 发送配置命令 (开启BSD/自动上报/获取版本) — 16-bit校验和
  - 帧解析 (0x5A上报/0x59回复/0x58命令)
  - 波特率扫描
"""

import serial
import serial.tools.list_ports
import time
import sys
import struct
from datetime import datetime

PORT = "COM7"
BAUD = 921600
DATA_BITS = 8
STOP_BITS = 1
PARITY = serial.PARITY_NONE

HEAD_REPORT = 0x5A
HEAD_REPLY  = 0x59
HEAD_CMD    = 0x58
TYPE_BSD    = 7

TYPE_NAMES = {
    0: "FMCW完整", 1: "HTM高度", 2: "电平探测",
    3: "运动存在", 4: "呼吸心率", 5: "分区检测",
    6: "简化BHR", 7: "BSD盲区",
}


def calc_checksum8(data: bytes) -> int:
    return sum(data) & 0xFF


def calc_checksum16(data: bytes) -> bytes:
    s = sum(data)
    return bytes([s & 0xFF, (s >> 8) & 0xFF])


def build_cmd(group: int, cmd: int, params: bytes = b"") -> bytes:
    cmd_byte = (group << 5) | cmd
    frame = bytes([HEAD_CMD, cmd_byte, len(params)]) + params
    return frame + calc_checksum16(frame)


def send_cmd(ser: serial.Serial, group: int, cmd: int, params: bytes = b"",
             name: str = "") -> bytes:
    """发送命令并等待回复"""
    frame = build_cmd(group, cmd, params)
    hex_str = " ".join(f"{b:02X}" for b in frame)
    label = f"  发送: {name} [{hex_str}]"
    print(label.ljust(100))
    ser.write(frame)
    ser.flush()
    time.sleep(0.3)
    reply = ser.read(ser.in_waiting or 128)
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
    return reply


def read_version(ser: serial.Serial):
    """组1 0xFE: 获取版本信息"""
    send_cmd(ser, 7, 0x1E, name="获取版本(0xFE)")


def read_status(ser: serial.Serial):
    """组1 0x01: 获取系统状态"""
    send_cmd(ser, 1, 0x01, name="获取状态(0x21)")


def enable_bsd(ser: serial.Serial):
    """组6 0xD1: 开启BSD感测功能"""
    send_cmd(ser, 6, 0x11, name="开启感测功能(0xD1)")


def enable_bsd_detection(ser: serial.Serial):
    """组6 0x10: 开启BSD检测"""
    send_cmd(ser, 6, 0x10, name="开启BSD检测(0xD0)")


def enable_auto_report(ser: serial.Serial):
    """组6 0x12: 开启自动上报"""
    send_cmd(ser, 6, 0x12, name="开启自动上报(0xD2)")


def set_baud(ser: serial.Serial, baud: int):
    """组0 0x19: 切换UART波特率
    baud index: 0=9600, 1=19200, 2=38400, 3=57600, 4=115200, 5=256000, 6=460800, 7=921600
    """
    idx_map = {9600:0, 19200:1, 38400:2, 57600:3, 115200:4, 256000:5, 460800:6, 921600:7}
    idx = idx_map.get(baud, 7)
    send_cmd(ser, 0, 0x19, bytes([idx]), name=f"切换波特率→{baud}")


def set_sense_time(ser: serial.Serial, ms: int = 100):
    """组0 0x04: 设置有効电平保持时间 (ms, little-endian 4字节)"""
    send_cmd(ser, 0, 0x04, struct.pack("<I", ms), name=f"电平保持时间={ms}ms")


# ─── 帧解析 ─────────────────────────────────────

def parse_frame(raw: bytes):
    head = raw[0]
    if head == HEAD_REPORT:
        msg_type = raw[2] if len(raw) > 2 else -1
        ok = (calc_checksum8(raw[:-1]) == raw[-1])
        status = "OK" if ok else "CK_ERR"
        tname = TYPE_NAMES.get(msg_type, f"T={msg_type}")
        if msg_type == TYPE_BSD and ok:
            payload = raw[3:-1]
            if len(payload) >= 4:
                obj_num = struct.unpack_from("<H", payload, 0)[0]
                obj_num = min(obj_num, 8)
                targets = []
                for i in range(obj_num):
                    off = 4 + i * 4
                    if off + 4 > len(payload):
                        break
                    rng, ang, vel, oid = struct.unpack_from("<bbbb", payload, off)
                    targets.append((rng, ang, vel, oid))
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{ts}] BSD | {obj_num}目标 | {status}")
                for r, a, v, o in targets:
                    print(f"  ID{o:+d} 距离{r:+d}m 角度{a:+d}° 速度{v:+d}m/s")
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


def parse_stream(buf: bytearray):
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
            garbage = buf[:idx]
            if len(garbage) >= 16:
                print(f"  [D] {len(garbage)}B: {' '.join(f'{b:02X}' for b in garbage[-16:])}")
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
        parse_frame(frame_raw)
        del buf[:frame_total]


# ─── 主逻辑 ─────────────────────────────────────

def open_port(port, baud):
    try:
        ser = serial.Serial(port=port, baudrate=baud, bytesize=8,
                            stopbits=1, parity=serial.PARITY_NONE, timeout=0.5)
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        return None


def monitor(port: str, baud: int, do_config: bool = True):
    print()
    print("=" * 70)
    print(f"  MS60-1211S80M-BSD | Port: {port} | Baud: {baud}")
    print("=" * 70)
    print()

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

    buf = bytearray()
    total = 0
    t0 = time.time()

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buf.extend(chunk)
                total += len(chunk)
                parse_stream(buf)
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(f"\n[DONE] 总计 {total}B | 运行 {time.time()-t0:.0f}s")


def baud_scan(port: str):
    """从低到高扫描"""
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
            if ser.in_waiting > 0:
                buf.extend(ser.read(ser.in_waiting))
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


if __name__ == "__main__":
    print(f"╔══════════════════════════════════════════════════════╗")
    print(f"║  Radar Debug v3  |  AT6010 HCI  |  {PORT} @ {BAUD}       ║")
    print(f"╚══════════════════════════════════════════════════════╝")

    if len(sys.argv) > 1:
        a = sys.argv[1]
        if a in ("-s", "--scan"):
            found = baud_scan(PORT)
            if found:
                monitor(PORT, found)
        elif a in ("-m", "--monitor"):
            monitor(PORT, BAUD, do_config=False)
        elif a.isdigit():
            monitor(PORT, int(a))
        else:
            print("用法: python radar_debug.py [-s|--scan] [-m|--monitor] [baud]")
    else:
        monitor(PORT, BAUD)
