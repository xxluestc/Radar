/*
 * MS60-1211S80M-BSD (AT6010) 雷达 BSD 监控程序 — 飞腾派 PE2204 版
 * 基于 radar_bsd.c (MP257版) 改写，仅修改串口设备路径和波特率
 * 雷达协议和配置命令与开发板无关，应用层代码完全一致
 *
 * 飞腾派串口映射:
 *   - J1 DEBUG_UART1 (Pin7=TXD, Pin9=RXD) → /dev/ttyAMA1 (0x2800D000) [console占用]
 *   - 40pin/J1 UART2 (Pin8=TXD, Pin10=RXD) → /dev/ttyAMA2 (0x2800E000)
 *
 * 编译 (交叉编译):
 *   export PATH=/home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin:$PATH
 *   aarch64-none-linux-gnu-gcc -Wall -O2 -o radar_phytium_bsd radar_phytium_bsd.c
 *
 * 运行:
 *   ./radar_phytium_bsd              默认 /dev/ttyAMA2 @ 921600
 *   ./radar_phytium_bsd -v           详细输出
 *   ./radar_phytium_bsd -d /dev/ttyAMA2 -b 921600
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>

#define DEFAULT_UART_DEVICE  "/dev/ttyAMA2"
#define DEFAULT_BAUDRATE     921600

#define HEAD_CMD    0x58
#define HEAD_REPLY  0x59
#define HEAD_REPORT 0x5A
#define TYPE_BSD    7

#define ALARM_WARN_DIST      15
#define ALARM_WARN_SPEED     -2
#define ALARM_DANGER_DIST    8
#define ALARM_DANGER_SPEED   -4
#define ALARM_CRITICAL_DIST  4
#define ALARM_CRITICAL_SPEED -1

#pragma pack(push, 1)
typedef struct {
    int8_t  range_val;
    int8_t  angle_val;
    int8_t  velo_val;
    int8_t  objId;
} bsd_obj_t;

typedef struct {
    uint16_t obj_num;
    uint16_t reserved;
    bsd_obj_t obj[8];
} bsd_det_t;
#pragma pack(pop)

static volatile int g_running = 1;
static int g_fd = -1;
static int g_verbose = 0;
static int g_warning_enabled = 1;
static FILE *g_log_file = NULL;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

static int set_uart(int fd, int baudrate)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "tcgetattr failed: %s\n", strerror(errno));
        return -1;
    }
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 115200: speed = B115200; break;
        case 921600: speed = B921600; break;
        default:
            fprintf(stderr, "Unsupported baudrate: %d\n", baudrate);
            return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST & ~ONLCR;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "tcsetattr failed: %s\n", strerror(errno));
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return 0;
}

static void get_timestamp(char *buf, size_t size)
{
    struct timeval tv;
    struct tm tm_info;
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm_info);
    snprintf(buf, size, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
             tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
             tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, tv.tv_usec / 1000);
}

static void log_printf(const char *fmt, ...)
{
    char ts[32];
    get_timestamp(ts, sizeof(ts));
    va_list args;
    va_start(args, fmt);
    fprintf(stdout, "[%s] ", ts);
    vfprintf(stdout, fmt, args);
    fflush(stdout);
    if (g_log_file) {
        fprintf(g_log_file, "[%s] ", ts);
        vfprintf(g_log_file, fmt, args);
        fflush(g_log_file);
    }
    va_end(args);
}

static uint16_t calc_sum16(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

static uint8_t calc_sum8(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

static int send_cmd(int fd, uint8_t group, uint8_t cmd, const uint8_t *params, int param_len)
{
    uint8_t frame[64];
    int idx = 0;
    frame[idx++] = HEAD_CMD;
    uint8_t cmd_byte = (group << 5) | (cmd & 0x1F);
    frame[idx++] = cmd_byte;
    frame[idx++] = (uint8_t)param_len;
    if (params && param_len > 0) {
        memcpy(&frame[idx], params, param_len);
        idx += param_len;
    }
    uint16_t csum = calc_sum16(frame, idx);
    frame[idx++] = (uint8_t)(csum & 0xFF);
    frame[idx++] = (uint8_t)((csum >> 8) & 0xFF);

    if (g_verbose) {
        printf("TX: ");
        for (int i = 0; i < idx; i++) printf("%02X ", frame[i]);
        printf("\n");
    }

    int written = write(fd, frame, idx);
    if (written != idx) {
        fprintf(stderr, "write failed: %s\n", strerror(errno));
        return -1;
    }
    tcdrain(fd);
    return 0;
}

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static const char *check_alarm(int dist, int speed)
{
    if (dist <= ALARM_CRITICAL_DIST && speed <= ALARM_CRITICAL_SPEED) return "CRITICAL";
    if (dist <= ALARM_DANGER_DIST    && speed <= ALARM_DANGER_SPEED)    return "DANGER";
    if (dist <= ALARM_WARN_DIST      && speed <= ALARM_WARN_SPEED)      return "WARN";
    return NULL;
}

static void process_bsd_report(const bsd_det_t *bsd)
{
    int obj_count = bsd->obj_num;
    if (obj_count > 8) obj_count = 8;

    log_printf("BSD: %d target(s) detected\n", obj_count);

    for (int i = 0; i < obj_count; i++) {
        const bsd_obj_t *o = &bsd->obj[i];
        const char *alarm = check_alarm(o->range_val, o->velo_val);
        log_printf("  ID=%d  Distance=%dm  Angle=%d  Speed=%dm/s%s%s\n",
                   o->objId, o->range_val, o->angle_val, o->velo_val,
                   alarm ? "  *** " : "", alarm ? alarm : "");
    }
}

static int process_report_frame(const uint8_t *payload, int payload_len)
{
    if (payload_len < 1) return -1;
    uint8_t type = payload[0];
    if (g_verbose) log_printf("Report TYPE=%d, len=%d\n", type, payload_len - 1);

    if (type != TYPE_BSD) return 0;

    int data_len = payload_len - 1;
    if (data_len < 4) {
        if (g_verbose) log_printf("BSD report too short: %d\n", data_len);
        return -1;
    }

    const uint8_t *data = payload + 1;
    bsd_det_t bsd;
    memset(&bsd, 0, sizeof(bsd));
    bsd.obj_num = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    int obj_count = bsd.obj_num;
    if (obj_count > 8) obj_count = 8;
    int expected_len = 4 + obj_count * (int)sizeof(bsd_obj_t);
    if (data_len < expected_len) {
        if (g_verbose) log_printf("BSD data incomplete: have %d, need %d\n", data_len, expected_len);
        obj_count = (data_len - 4) / (int)sizeof(bsd_obj_t);
        if (obj_count < 0) obj_count = 0;
    }
    for (int i = 0; i < obj_count; i++) {
        int off = 4 + i * (int)sizeof(bsd_obj_t);
        if (off + (int)sizeof(bsd_obj_t) <= data_len) {
            memcpy(&bsd.obj[i], &data[off], sizeof(bsd_obj_t));
        }
    }
    process_bsd_report(&bsd);
    return 0;
}

static void process_reply_frame(const uint8_t *payload, int payload_len)
{
    if (payload_len < 3) return;
    uint8_t cmd_byte = payload[0];
    uint8_t param_len = payload[1];
    const uint8_t *params = &payload[2];
    int grp = cmd_byte >> 5;
    int cmd = cmd_byte & 0x1F;

    if (g_verbose) {
        log_printf("Reply G%d.0x%02X Len=%d: ", grp, cmd, param_len);
        for (int i = 0; i < param_len && i < payload_len - 2; i++) printf("%02X ", params[i]);
        printf("\n");
    }

    if (grp == 7 && cmd == 0x1E) {
        if (param_len >= 8) {
            log_printf("Version: SW=%d.%d.%d  Customer=%d.%d  CI=%d.%d  Algo=%d\n",
                       params[0], params[1], params[2], params[3], params[4],
                       params[5], params[6], params[7]);
        }
    } else if (grp == 6 && cmd == 0x11) {
        log_printf("Sensor enable: %s\n", param_len >= 1 && params[0] == 0 ? "FAIL" : "OK");
    } else if (grp == 6 && cmd == 0x10) {
        log_printf("BSD detection: %s\n", param_len >= 1 && params[0] == 0 ? "FAIL" : "OK");
    } else if (grp == 6 && cmd == 0x12) {
        log_printf("Auto report: %s\n", param_len >= 1 && params[0] == 0 ? "FAIL" : "OK");
    } else if (grp == 1 && cmd == 0x01) {
        log_printf("Status: %s\n", param_len >= 1 ? (params[0] ? "ON" : "OFF") : "UNKNOWN");
    }
}

static int process_frame(const uint8_t *frame, int frame_len)
{
    if (frame_len < 4) return -1;
    uint8_t head = frame[0];

    if (head == HEAD_REPORT) {
        uint8_t len = frame[1];
        if (2 + len + 1 > frame_len) return -1;
        uint8_t cs = calc_sum8(frame, 2 + len);
        if (cs != frame[2 + len]) {
            if (g_verbose) log_printf("Report CS error: calc=0x%02X recv=0x%02X\n", cs, frame[2 + len]);
            return -1;
        }
        return process_report_frame(&frame[2], len + 1);
    } else if (head == HEAD_REPLY) {
        if (frame_len < 5) return -1;
        uint8_t cmd_byte = frame[1];
        uint8_t param_len = frame[2];
        if (3 + param_len + 2 > frame_len) return -1;
        uint16_t recv_cs = (uint16_t)frame[3 + param_len] | ((uint16_t)frame[3 + param_len + 1] << 8);
        uint16_t calc_cs = calc_sum16(frame, 3 + param_len);
        if (calc_cs != recv_cs) {
            if (g_verbose) log_printf("Reply CS error: calc=0x%04X recv=0x%04X\n", calc_cs, recv_cs);
            return -1;
        }
        uint8_t payload[256];
        payload[0] = cmd_byte;
        payload[1] = param_len;
        if (param_len > 0) memcpy(&payload[2], &frame[3], param_len);
        process_reply_frame(payload, 2 + param_len);
        return 0;
    }
    return -1;
}

static void flush_rx(int fd)
{
    uint8_t tmp[256];
    int total = 0;
    int n;
    while ((n = read(fd, tmp, sizeof(tmp))) > 0) total += n;
    if (g_verbose && total > 0) log_printf("Flushed %d bytes\n", total);
    (void)total;
}

static int wait_reply(int fd, int timeout_ms)
{
    uint8_t buf[512];
    int buf_len = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (g_running) {
        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= timeout_ms) break;

        int remain = timeout_ms - elapsed;
        struct timeval tv = { remain / 1000, (remain % 1000) * 1000 };
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret <= 0) break;
        if (!FD_ISSET(fd, &rfds)) break;

        int n = read(fd, buf + buf_len, sizeof(buf) - buf_len);
        if (n <= 0) break;
        buf_len += n;

        while (buf_len >= 4) {
            uint8_t head = buf[0];
            int frame_total = -1;
            if (head == HEAD_REPLY) {
                if (buf_len < 3) break;
                uint8_t plen = buf[2];
                frame_total = 5 + plen;
            } else if (head == HEAD_REPORT) {
                if (buf_len < 3) break;
                uint8_t plen = buf[1];
                frame_total = 3 + plen + 1;
            } else {
                memmove(buf, buf + 1, buf_len - 1);
                buf_len--;
                continue;
            }
            if (frame_total > (int)sizeof(buf) || frame_total > buf_len) break;

            int consumed = frame_total;
            uint8_t frame[512];
            if (consumed <= 512) {
                memcpy(frame, buf, consumed);
                process_frame(frame, consumed);
            }
            memmove(buf, buf + consumed, buf_len - consumed);
            buf_len -= consumed;
        }
    }
    return 0;
}

static int send_with_reply(int fd, uint8_t grp, uint8_t cmd, const uint8_t *params, int plen, int timeout_ms)
{
    flush_rx(fd);
    if (send_cmd(fd, grp, cmd, params, plen) != 0) return -1;
    return wait_reply(fd, timeout_ms);
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("Options:\n");
    printf("  -d DEVICE   UART device (default: %s)\n", DEFAULT_UART_DEVICE);
    printf("  -b BAUD     Baud rate (default: %d)\n", DEFAULT_BAUDRATE);
    printf("  -v          Verbose output\n");
    printf("  -l FILE     Log to file\n");
    printf("  -n          Disable warning alerts\n");
    printf("  -h          Show this help\n");
    printf("\nAlarm thresholds:\n");
    printf("  WARN      : distance <= %dm && speed <= %dm/s\n", ALARM_WARN_DIST, ALARM_WARN_SPEED);
    printf("  DANGER    : distance <= %dm && speed <= %dm/s\n", ALARM_DANGER_DIST, ALARM_DANGER_SPEED);
    printf("  CRITICAL  : distance <= %dm && speed <= %dm/s\n", ALARM_CRITICAL_DIST, ALARM_CRITICAL_SPEED);
}

int main(int argc, char *argv[])
{
    const char *device = DEFAULT_UART_DEVICE;
    int baudrate = DEFAULT_BAUDRATE;
    const char *logfile = NULL;

    int opt;
    while ((opt = getopt(argc, argv, "d:b:vl:nh")) != -1) {
        switch (opt) {
            case 'd': device = optarg; break;
            case 'b': baudrate = atoi(optarg); break;
            case 'v': g_verbose = 1; break;
            case 'l': logfile = optarg; break;
            case 'n': g_warning_enabled = 0; break;
            default:
            case 'h': print_usage(argv[0]); return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (logfile) {
        g_log_file = fopen(logfile, "a");
        if (!g_log_file) {
            fprintf(stderr, "Cannot open log file: %s\n", strerror(errno));
            return 1;
        }
    }

    log_printf("Radar BSD starting: %s @ %d\n", device, baudrate);
    if (!g_warning_enabled) log_printf("Warning alerts DISABLED\n");

    g_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", device, strerror(errno));
        return 1;
    }

    if (set_uart(g_fd, baudrate) != 0) {
        close(g_fd);
        return 1;
    }
    log_printf("UART configured\n");

    log_printf("Step 1: Send init commands...\n");
    send_with_reply(g_fd, 7, 0x1E, NULL, 0, 1000);
    send_with_reply(g_fd, 1, 0x01, NULL, 0, 1000);

    log_printf("Step 2: Enable sensor, BSD detection, auto report...\n");
    send_with_reply(g_fd, 6, 0x11, (uint8_t[]){0x01}, 1, 1000);
    send_with_reply(g_fd, 6, 0x10, (uint8_t[]){0x01}, 1, 1000);
    send_with_reply(g_fd, 6, 0x12, (uint8_t[]){0x01}, 1, 1000);
    flush_rx(g_fd);
    usleep(5000000);
    flush_rx(g_fd);

    log_printf("READY — Entering main loop, waiting for BSD reports...\n\n");

    uint8_t rx_buf[512];
    int rx_len = 0;

    while (g_running) {
        fd_set rfds;
        struct timeval tv = {1, 0};
        FD_ZERO(&rfds);
        FD_SET(g_fd, &rfds);
        int ret = select(g_fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret > 0 && FD_ISSET(g_fd, &rfds)) {
            int n = read(g_fd, rx_buf + rx_len, sizeof(rx_buf) - rx_len);
            if (n > 0) rx_len += n;

            if (g_verbose && n > 0) {
                log_printf("RX raw (%d): ", n);
                for (int i = rx_len - n; i < rx_len; i++) printf("%02X ", rx_buf[i]);
                printf("\n");
            }

            while (rx_len >= 4) {
                uint8_t head = rx_buf[0];
                int frame_total = -1;
                if (head == HEAD_REPORT) {
                    frame_total = 3 + rx_buf[1] + 1;
                } else if (head == HEAD_REPLY) {
                    if (rx_len < 3) break;
                    frame_total = 5 + rx_buf[2];
                } else {
                    int i = 1;
                    for (; i < rx_len; i++) {
                        if (rx_buf[i] == HEAD_REPORT || rx_buf[i] == HEAD_REPLY || rx_buf[i] == HEAD_CMD)
                            break;
                    }
                    if (i < rx_len) {
                        memmove(rx_buf, rx_buf + i, rx_len - i);
                        rx_len -= i;
                    } else {
                        rx_len = 0;
                    }
                    continue;
                }
                if (frame_total > (int)sizeof(rx_buf) || frame_total > rx_len) break;

                uint8_t frame[512];
                memcpy(frame, rx_buf, frame_total);
                process_frame(frame, frame_total);
                memmove(rx_buf, rx_buf + frame_total, rx_len - frame_total);
                rx_len -= frame_total;
            }
            if (rx_len >= (int)sizeof(rx_buf)) rx_len = 0;
        }
    }

    log_printf("Radar BSD stopped\n");
    if (g_log_file) fclose(g_log_file);
    close(g_fd);
    return 0;
}
