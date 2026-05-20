"""
MS60-1211S80M-BSD (AT6010) 雷达USB转TTL调试工具 v4
支持:
  - 发送配置命令 (开启BSD/自动上报/获取版本) — 16-bit校验和
  - 帧解析 (0x5A上报/0x59回复/0x58命令)
  - 波特率扫描
  - CSV数据记录 (每次运行自动保存)
  - 实时可视化 (鸟瞰图 + 距离/速度曲线 + 报警区域)
  - 可配置报警阈值 (距离 + 接近速度)
"""

import serial
import serial.tools.list_ports
import time
import sys
import struct
import csv
import os
import math
from datetime import datetime

PORT = "COM9"
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
    send_cmd(ser, 7, 0x1E, name="获取版本(0xFE)")


def read_status(ser: serial.Serial):
    send_cmd(ser, 1, 0x01, name="获取状态(0x21)")


def enable_bsd(ser: serial.Serial):
    send_cmd(ser, 6, 0x11, bytes([0x01]), name="开启感测功能(0xD1)")


def enable_bsd_detection(ser: serial.Serial):
    send_cmd(ser, 6, 0x10, bytes([0x01]), name="开启BSD检测(0xD0)")


def enable_auto_report(ser: serial.Serial):
    send_cmd(ser, 6, 0x12, bytes([0x01]), name="开启自动上报(0xD2)")


def set_baud(ser: serial.Serial, baud: int):
    idx_map = {9600:0, 19200:1, 38400:2, 57600:3, 115200:4, 256000:5, 460800:6, 921600:7}
    idx = idx_map.get(baud, 7)
    send_cmd(ser, 0, 0x19, bytes([idx]), name=f"切换波特率→{baud}")


def set_sense_time(ser: serial.Serial, ms: int = 100):
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


class RadarVisualizer:
    def __init__(self):
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
            from matplotlib.animation import FuncAnimation
            self.plt = plt
            self.patches = patches
            self.FuncAnimation = FuncAnimation
            self.available = True
        except ImportError:
            self.available = False
            return

        self.fig = plt.figure(figsize=(14, 8))
        self.fig.suptitle("MS60-1211S80M-BSD BSD Monitor", fontsize=14, fontweight="bold")

        self.ax_spatial = self.fig.add_subplot(121)
        self.ax_dist = self.fig.add_subplot(222)
        self.ax_speed = self.fig.add_subplot(224)

        self.history = {}
        self.max_history = 300
        self.targets_now = []

        self._setup_spatial()
        self._setup_timeseries(self.ax_dist, "Distance (m)", "m")
        self._setup_timeseries(self.ax_speed, "Speed (m/s)", "m/s")

        self.fig.tight_layout(rect=[0, 0, 1, 0.95])

    def _setup_spatial(self):
        ax = self.ax_spatial
        ax.set_xlim(-20, 20)
        ax.set_ylim(-1, 20)
        ax.set_xlabel("Lateral (m)")
        ax.set_ylabel("Distance behind (m)")
        ax.set_title("Bird's Eye View (radar at bottom)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

        ax.plot(0, 0, "k^", markersize=15, label="Radar")
        ax.axhline(y=0, color="k", linewidth=2)

        for level, d_thresh, s_thresh, color in ALARM_LEVELS:
            fan_angles = [-40, 40]
            theta1, theta2 = 90 - fan_angles[1], 90 - fan_angles[0]
            wedge = self.patches.Wedge((0, 0), d_thresh, theta1, theta2,
                                        alpha=0.1, color=color.strip("\033[").replace("91m", "red").replace("93m", "orange").replace("96m", "cyan"))
            ax.add_patch(wedge)
            ax.text(d_thresh * 0.7, d_thresh * 0.3, f"{level}\n{d_thresh}m",
                    fontsize=7, alpha=0.5, ha="center")

    def _setup_timeseries(self, ax, title, unit):
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(unit)
        ax.grid(True, alpha=0.3)

    def update(self, targets, elapsed):
        self.targets_now = targets
        for t in targets:
            oid = t["id"]
            if oid not in self.history:
                self.history[oid] = {"t": [], "dist": [], "speed": [], "angle": []}
            h = self.history[oid]
            h["t"].append(elapsed)
            h["dist"].append(t["dist"])
            h["speed"].append(t["speed"])
            h["angle"].append(t["angle"])
            if len(h["t"]) > self.max_history:
                for k in h:
                    h[k] = h[k][-self.max_history:]

    def draw(self):
        if not self.available:
            return

        self.ax_spatial.collections.clear()
        for p in list(self.ax_spatial.patches):
            if isinstance(p, self.plt.Circle):
                p.remove()

        for t in self.targets_now:
            dist = t["dist"]
            angle = t["angle"]
            speed = t["speed"]
            oid = t["id"]

            x = dist * math.sin(math.radians(angle))
            y = dist * math.cos(math.radians(angle))

            alarm, color_code = check_alarm(dist, speed)
            if alarm == "CRITICAL":
                c = "red"
                ms = 12
            elif alarm == "DANGER":
                c = "orange"
                ms = 10
            elif alarm == "WARN":
                c = "cyan"
                ms = 8
            else:
                c = "green"
                ms = 6

            self.ax_spatial.plot(x, y, "o", color=c, markersize=ms, alpha=0.8)
            self.ax_spatial.annotate(f"ID{oid} {dist}m {speed}m/s",
                                      (x, y), textcoords="offset points",
                                      xytext=(5, 5), fontsize=7, color=c)

        self.ax_dist.cla()
        self.ax_speed.cla()
        self._setup_timeseries(self.ax_dist, "Distance (m)", "m")
        self._setup_timeseries(self.ax_speed, "Speed (m/s)", "m/s")

        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
                  "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"]

        for oid, h in self.history.items():
            c = colors[oid % len(colors)]
            label = f"ID{oid}"
            self.ax_dist.plot(h["t"], h["dist"], "-", color=c, label=label, linewidth=1)
            self.ax_speed.plot(h["t"], h["speed"], "-", color=c, label=label, linewidth=1)

        self.ax_dist.axhline(y=ALARM_WARN_DIST, color="cyan", linestyle="--", alpha=0.3, label=f"WARN {ALARM_WARN_DIST}m")
        self.ax_dist.axhline(y=ALARM_DANGER_DIST, color="orange", linestyle="--", alpha=0.3, label=f"DANGER {ALARM_DANGER_DIST}m")
        self.ax_dist.axhline(y=ALARM_CRITICAL_DIST, color="red", linestyle="--", alpha=0.3, label=f"CRIT {ALARM_CRITICAL_DIST}m")

        self.ax_speed.axhline(y=ALARM_WARN_SPEED, color="cyan", linestyle="--", alpha=0.3, label=f"WARN {ALARM_WARN_SPEED}m/s")
        self.ax_speed.axhline(y=ALARM_DANGER_SPEED, color="orange", linestyle="--", alpha=0.3, label=f"DANGER {ALARM_DANGER_SPEED}m/s")
        self.ax_speed.axhline(y=0, color="gray", linestyle="-", alpha=0.3)

        if self.history:
            self.ax_dist.legend(fontsize=6, loc="upper right")
            self.ax_speed.legend(fontsize=6, loc="upper right")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def show(self):
        if self.available:
            self.plt.ion()
            self.plt.show(block=False)

    def close(self):
        if self.available:
            self.plt.close(self.fig)


def parse_frame(raw: bytes, logger=None, viz=None, elapsed=0):
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
                if viz:
                    viz.update(targets, elapsed)
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


def parse_stream(buf: bytearray, logger=None, viz=None, elapsed=0):
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
        parse_frame(frame_raw, logger, viz, elapsed)
        del buf[:frame_total]


def open_port(port, baud):
    try:
        ser = serial.Serial(port=port, baudrate=baud, bytesize=8,
                            stopbits=1, parity=serial.PARITY_NONE, timeout=0.5)
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        return None


def print_alarm_config():
    print("  报警阈值配置:")
    for level, d, s, _ in ALARM_LEVELS:
        print(f"    {level:8s}: 距离 ≤ {d:2d}m  且  速度 ≤ {s:+d}m/s (接近)")
    print()


def monitor(port: str, baud: int, do_config: bool = True,
            use_viz: bool = False, log_file: str = None):
    print()
    print("=" * 70)
    print(f"  MS60-1211S80M-BSD | Port: {port} | Baud: {baud}")
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

    viz = None
    if use_viz:
        viz = RadarVisualizer()
        if viz.available:
            viz.show()
            print("[VIZ] 可视化窗口已打开\n")
        else:
            print("[VIZ] matplotlib 不可用，跳过可视化\n")
            viz = None

    buf = bytearray()
    total = 0
    t0 = time.time()
    last_draw = t0

    try:
        while True:
            elapsed = time.time() - t0
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buf.extend(chunk)
                total += len(chunk)
                parse_stream(buf, logger, viz, elapsed)
            else:
                if viz and time.time() - last_draw > 0.2:
                    viz.draw()
                    last_draw = time.time()
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        fname, count = logger.close()
        if viz:
            viz.close()
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


def replay_csv(filename, use_viz=False):
    print(f"\n回放: {filename}")
    print("=" * 70)
    print_alarm_config()

    viz = None
    if use_viz:
        viz = RadarVisualizer()
        if viz.available:
            viz.show()
        else:
            viz = None

    with open(filename, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        print("无数据")
        return

    print(f"共 {len(rows)} 条记录，回放中...\n")

    prev_t = float(rows[0]["elapsed_s"])
    for row in rows:
        t = float(row["elapsed_s"])
        dt = t - prev_t
        if dt > 0:
            time.sleep(min(dt, 0.1))
        prev_t = t

        target = {
            "id": int(row["obj_id"]),
            "dist": int(row["dist_m"]),
            "angle": int(row["angle_deg"]),
            "speed": int(row["speed_ms"]),
        }
        alarm, color = check_alarm(target["dist"], target["speed"])
        alarm_str = f" {color}<<< {alarm} >>>\033[0m" if alarm else ""
        ts = row["timestamp"]
        print(f"[{ts}] ID{target['id']:+d} 距离{target['dist']:+d}m "
              f"角度{target['angle']:+d}° 速度{target['speed']:+d}m/s{alarm_str}")

        if viz:
            viz.update([target], t)
            viz.draw()

    if viz:
        viz.close()
    print(f"\n回放完成")


def print_usage():
    print("用法:")
    print("  python radar_debug.py                    默认: 配置BSD + 监控 + CSV记录")
    print("  python radar_debug.py -v                 可视化模式 (鸟瞰图 + 曲线)")
    print("  python radar_debug.py -m                 仅监控 (不发送配置命令)")
    print("  python radar_debug.py -s                 波特率扫描")
    print("  python radar_debug.py -l FILE            指定CSV日志文件名")
    print("  python radar_debug.py -r FILE            回放CSV日志")
    print("  python radar_debug.py -rv FILE           回放CSV日志 + 可视化")
    print("  python radar_debug.py 115200             指定波特率")
    print()
    print("报警阈值 (修改脚本顶部常量):")
    for level, d, s, _ in ALARM_LEVELS:
        print(f"  {level:8s}: 距离 ≤ {d}m  且  速度 ≤ {s}m/s")


if __name__ == "__main__":
    print(f"╔══════════════════════════════════════════════════════╗")
    print(f"║  Radar Debug v4  |  AT6010 HCI  |  {PORT} @ {BAUD}       ║")
    print(f"╚══════════════════════════════════════════════════════╝")

    args = sys.argv[1:]
    use_viz = False
    do_config = True
    scan_mode = False
    monitor_only = False
    log_file = None
    replay_file = None
    custom_baud = None

    i = 0
    while i < len(args):
        a = args[i]
        if a in ("-v", "--visualize"):
            use_viz = True
        elif a in ("-m", "--monitor"):
            monitor_only = True
        elif a in ("-s", "--scan"):
            scan_mode = True
        elif a in ("-l", "--log"):
            i += 1
            if i < len(args):
                log_file = args[i]
        elif a in ("-r", "--replay"):
            i += 1
            if i < len(args):
                replay_file = args[i]
        elif a in ("-h", "--help"):
            print_usage()
            sys.exit(0)
        elif a.isdigit():
            custom_baud = int(a)
        i += 1

    if replay_file:
        replay_csv(replay_file, use_viz=use_viz)
    elif scan_mode:
        found = baud_scan(PORT)
        if found:
            monitor(PORT, found, use_viz=use_viz, log_file=log_file)
    else:
        baud = custom_baud or BAUD
        monitor(PORT, baud, do_config=not monitor_only,
                use_viz=use_viz, log_file=log_file)
