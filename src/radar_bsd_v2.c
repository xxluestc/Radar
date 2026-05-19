#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <stdarg.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#define DEFAULT_UART_DEVICE "/dev/ttySTM3"
#define DEFAULT_BAUDRATE 921600

#define FRAME_HEAD_CMD 0x58
#define FRAME_HEAD_RESP 0x59
#define FRAME_HEAD_REPORT 0x5A
#define REPORT_TYPE_BSD 7
#define MAX_BSD_OBJECTS 8

static volatile int g_running = 1;
static int g_fd = -1;
static int g_verbose = 0;
static int g_warning_enabled = 1;
static int g_warning_distance = 5;
static int g_warning_approach_speed = 2;
static FILE *g_log_file = NULL;

static void signal_handler(int sig) { g_running = 0; }

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

static void log_msg(const char *fmt, ...)
{
    char ts[64];
    get_timestamp(ts, sizeof(ts));
    va_list args;
    va_start(args, fmt);
    fprintf(stdout, "[%s] ", ts);
    vfprintf(stdout, fmt, args);
    if (g_log_file) {
        fprintf(g_log_file, "[%s] ", ts);
        vfprintf(g_log_file, fmt, args);
        fflush(g_log_file);
    }
    va_end(args);
}

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static int set_uart(int fd, int baudrate)
{
    struct termios tty;
    tcgetattr(fd, &tty);
    speed_t speed = B921600;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 115200: speed = B115200; break;
        case 921600: speed = B921600; break;
        default: return -1;
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
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    return 0;
}

static uint16_t calc_checksum(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

static uint8_t calc_report_checksum(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

static int send_cmd(int fd, uint8_t cmd_byte, const uint8_t *params, int param_len)
{
    uint8_t frame[64];
    int idx = 0;
    frame[idx++] = FRAME_HEAD_CMD;
    frame[idx++] = cmd_byte;
    frame[idx++] = (uint8_t)param_len;
    if (params && param_len > 0) {
        memcpy(&frame[idx], params, param_len);
        idx += param_len;
    }
    uint16_t checksum = calc_checksum(frame, idx);
    frame[idx++] = (uint8_t)(checksum & 0xFF);
    frame[idx++] = (uint8_t)((checksum >> 8) & 0xFF);
    write(fd, frame, idx);
    tcdrain(fd);
    if (g_verbose) {
        printf("TX: ");
        print_hex(frame, idx);
        printf("\n");
    }
    return 0;
}

static int recv_bytes(int fd, uint8_t *buf, int max_len, int timeout_ms)
{
    int total = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);
    while (g_running) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, max_len - total);
            if (n > 0) total += n;
            if (total >= max_len) break;
        }
        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= timeout_ms) break;
    }
    return total;
}

static int send_and_recv(int fd, uint8_t cmd_byte, const uint8_t *params, int param_len,
                         uint8_t *resp, int resp_max, int timeout_ms)
{
    uint8_t tmp[256];
    while (read(fd, tmp, sizeof(tmp)) > 0) {}

    send_cmd(fd, cmd_byte, params, param_len);
    tcdrain(fd);

    usleep(50000);

    return recv_bytes(fd, resp, resp_max, timeout_ms);
}

static void do_parse_payload(uint8_t cmd_byte, const uint8_t *params, int param_len)
{
    if (cmd_byte == 0xFE) {
        if (param_len >= 8) {
            log_msg("Version: SW=%d.%d.%d, Customer=%d.%d, CI=%d.%d, Algo=%d\n",
                   params[0], params[1], params[2],
                   params[3], params[4],
                   params[5], params[6],
                   params[7]);
        }
    } else if (cmd_byte == 0xD0) {
        if (param_len >= 1)
            log_msg("Radar state: %s\n", params[0] ? "ON" : "OFF");
    } else if (cmd_byte == 0xD2) {
        if (param_len >= 1)
            log_msg("Algorithm type: %d\n", params[0]);
    } else if (cmd_byte == 0xD1) {
        if (param_len >= 1)
            log_msg("Radar enable: %s\n", params[0] == 0x00 ? "SUCCESS" : "FAILED");
    } else if (cmd_byte == 0x90) {
        if (param_len >= 1)
            log_msg("ULP active time set: %s\n", params[0] == 0x00 ? "SUCCESS" : "FAILED");
    } else if (cmd_byte == 0x13) {
        if (param_len >= 1)
            log_msg("System Reset: %s\n", params[0] == 0x00 ? "SUCCESS" : "FAILED");
    }
}

static int check_frame(const uint8_t *frame, int frame_len)
{
    if (frame_len < 5) return -1;
    uint8_t head = frame[0];
    if (head != FRAME_HEAD_RESP && head != FRAME_HEAD_REPORT) return -1;

    if (head == FRAME_HEAD_RESP) {
        uint8_t plen = frame[2];
        int expected = 3 + plen + 2;
        if (expected != frame_len) return -1;
        uint16_t recv_cksum = frame[3 + plen] | (frame[3 + plen + 1] << 8);
        uint16_t calc_cksum = calc_checksum(frame, 3 + plen);
        if (calc_cksum != recv_cksum) return -1;
        do_parse_payload(frame[1], &frame[3], plen);
        return 0;
    }
    return -1;
}

static int try_parse_response(uint8_t cmd_byte, const uint8_t *data, int data_len)
{
    if (data_len == 0) return -1;

    if (data[0] == FRAME_HEAD_RESP) {
        if (check_frame(data, data_len) == 0) {
            printf("  [Complete frame] ");
            print_hex(data, data_len);
            printf("\n");
            return 0;
        }
    }

    for (int lost = 1; lost <= 5; lost++) {
        uint8_t frame[256];
        int frame_len;

        switch (lost) {
            case 1:
                frame[0] = FRAME_HEAD_RESP;
                memcpy(&frame[1], data, data_len);
                frame_len = 1 + data_len;
                break;
            case 2:
                frame[0] = FRAME_HEAD_RESP;
                frame[1] = cmd_byte;
                memcpy(&frame[2], data, data_len);
                frame_len = 2 + data_len;
                break;
            default:
                frame[0] = FRAME_HEAD_RESP;
                frame[1] = cmd_byte;
                if (data_len >= lost - 2) {
                    frame[2] = (uint8_t)(lost - 2);
                    memcpy(&frame[3], data, data_len);
                    frame_len = 2 + data_len;
                } else {
                    continue;
                }
                break;
        }

        if (frame_len < 5 || frame_len > (int)sizeof(frame)) continue;

        uint8_t plen = frame[2];
        int expected = 3 + plen + 2;
        if (expected != frame_len) continue;

        uint16_t recv_cksum = frame[3 + plen] | (frame[3 + plen + 1] << 8);
        uint16_t calc_cksum = calc_checksum(frame, 3 + plen);

        if (calc_cksum == recv_cksum) {
            printf("  [Repaired: lost %d byte(s)] ", lost);
            print_hex(frame, frame_len);
            printf("\n");
            do_parse_payload(frame[1], &frame[3], plen);
            return 0;
        }
    }

    for (int skip = 0; skip <= 2; skip++) {
        const uint8_t *d = data + skip;
        int dlen = data_len - skip;
        if (dlen <= 0) continue;

        for (int plen_guess = 0; plen_guess <= 32; plen_guess++) {
            int expected_total = 3 + plen_guess + 2;
            int lost = expected_total - (dlen + skip);
            if (lost < 1 || lost > 8) continue;

            uint8_t frame[256];
            frame[0] = FRAME_HEAD_RESP;
            frame[1] = cmd_byte;
            frame[2] = (uint8_t)plen_guess;

            int param_bytes_from_data = plen_guess;
            int total_lost_from_params = lost - 2;
            if (total_lost_from_params > 0) {
                param_bytes_from_data -= total_lost_from_params;
            }
            if (param_bytes_from_data < 0) param_bytes_from_data = 0;

            if (param_bytes_from_data + 2 > dlen) continue;

            memcpy(&frame[3], d, param_bytes_from_data);

            int cksum_offset = 3 + plen_guess;
            int data_cksum_offset = param_bytes_from_data;
            if (data_cksum_offset + 2 > dlen) continue;

            frame[cksum_offset] = d[data_cksum_offset];
            frame[cksum_offset + 1] = d[data_cksum_offset + 1];

            int frame_len = 3 + plen_guess + 2;

            uint16_t recv_cksum = frame[cksum_offset] | (frame[cksum_offset + 1] << 8);
            uint16_t calc_cksum = calc_checksum(frame, 3 + plen_guess);

            if (calc_cksum == recv_cksum) {
                printf("  [Repaired: skip=%d, lost=%d, plen=%d] ", skip, lost, plen_guess);
                print_hex(frame, frame_len);
                printf("\n");
                printf("  [DEBUG: cmd=0x%02X, params[0]=0x%02X, plen=%d]\n", frame[1], frame[3], plen_guess);
                do_parse_payload(frame[1], &frame[3], plen_guess);
                return 0;
            }
        }
    }

    printf("  [Could not repair, raw: ");
    print_hex(data, data_len);
    printf("]\n");
    return -1;
}

#pragma pack(push, 1)
typedef struct {
    int8_t range_val;
    int8_t angle_val;
    int8_t velo_val;
    int8_t objId;
} bsd_obj_info_t;
#pragma pack(pop)

static void process_bsd_data(const uint8_t *data, int data_len)
{
    if (data_len < 2) return;
    int obj_num = data[0] | (data[1] << 8);
    if (obj_num > MAX_BSD_OBJECTS) obj_num = MAX_BSD_OBJECTS;
    int expected = 4 + obj_num * 4;
    if (data_len < expected) {
        obj_num = (data_len - 4) / 4;
        if (obj_num < 0) obj_num = 0;
    }

    log_msg("BSD Report: %d target(s)\n", obj_num);
    int danger = 0;
    for (int i = 0; i < obj_num; i++) {
        int off = 4 + i * 4;
        if (off + 4 > data_len) break;
        int8_t dist = (int8_t)data[off];
        int8_t angle = (int8_t)data[off + 1];
        int8_t speed = (int8_t)data[off + 2];
        int8_t id = (int8_t)data[off + 3];
        log_msg("  Target[%d]: ID=%d, Dist=%dm, Angle=%d, Speed=%dm/s\n", i, id, dist, angle, speed);
        if (g_warning_enabled && dist > 0 && dist <= g_warning_distance && speed > g_warning_approach_speed) {
            danger = 1;
            log_msg("  WARNING: Target approaching! Dist=%dm, Speed=%dm/s\n", dist, speed);
        }
    }
    if (danger) log_msg("DANGER ALERT!\n");
}

static int try_parse_report(const uint8_t *data, int data_len)
{
    for (int lost = 0; lost <= 2; lost++) {
        uint8_t frame[256];
        int frame_len = lost + data_len;
        if (frame_len > (int)sizeof(frame)) continue;

        frame[0] = FRAME_HEAD_REPORT;
        if (lost >= 1) {
            memcpy(&frame[1], data, data_len);
            frame_len = 1 + data_len;
        } else {
            memcpy(&frame, data, data_len);
            frame_len = data_len;
        }

        if (frame_len < 3) continue;

        uint8_t len = frame[1];
        int expected = 2 + len + 1;
        if (expected != frame_len) continue;

        uint8_t calc_ck = calc_report_checksum(frame, 2 + len);
        if (calc_ck == frame[2 + len]) {
            printf("  [Report repaired: lost %d byte(s)] ", lost);
            print_hex(frame, frame_len);
            printf("\n");

            if (len >= 1 && frame[2] == REPORT_TYPE_BSD) {
                process_bsd_data(&frame[3], len - 1);
            }
            return 0;
        }
    }
    return -1;
}

int main(int argc, char *argv[])
{
    const char *device = DEFAULT_UART_DEVICE;
    int baudrate = DEFAULT_BAUDRATE;
    const char *logfile = NULL;

    int opt;
    while ((opt = getopt(argc, argv, "d:b:vl:W:S:nh")) != -1) {
        switch (opt) {
            case 'd': device = optarg; break;
            case 'b': baudrate = atoi(optarg); break;
            case 'v': g_verbose = 1; break;
            case 'l': logfile = optarg; break;
            case 'W': g_warning_distance = atoi(optarg); break;
            case 'S': g_warning_approach_speed = atoi(optarg); break;
            case 'n': g_warning_enabled = 0; break;
            case 'h':
                printf("Usage: %s [-d device] [-b baud] [-v] [-l log] [-W dist] [-S speed] [-n]\n", argv[0]);
                return 0;
            default: return 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (logfile) {
        g_log_file = fopen(logfile, "a");
    }

    log_msg("Radar BSD v2 - Header Loss Compensation\n");
    log_msg("Device: %s, Baudrate: %d\n", device, baudrate);

    g_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", device, strerror(errno));
        return 1;
    }
    if (set_uart(g_fd, baudrate) != 0) {
        fprintf(stderr, "Failed to configure UART\n");
        close(g_fd);
        return 1;
    }
    log_msg("UART configured\n");

    log_msg("Pre-warming UART...\n");
    {
        uint8_t tmp[256];
        while (read(g_fd, tmp, sizeof(tmp)) > 0) {}
    }
    usleep(500000);

    uint8_t rx[256];
    int rx_len;

    log_msg("Step 1: Set ULP active time to 255s\n");
    rx_len = send_and_recv(g_fd, 0x90, (uint8_t[]){0xFF}, 1, rx, sizeof(rx), 1000);
    if (g_verbose || rx_len > 0) {
        printf("  RX (%d bytes): ", rx_len);
        print_hex(rx, rx_len);
        printf("\n");
    }
    if (rx_len > 0) try_parse_response(0x90, rx, rx_len);
    usleep(200000);

    log_msg("Step 2: Get version\n");
    rx_len = send_and_recv(g_fd, 0xFE, NULL, 0, rx, sizeof(rx), 1000);
    if (g_verbose || rx_len > 0) {
        printf("  RX (%d bytes): ", rx_len);
        print_hex(rx, rx_len);
        printf("\n");
    }
    if (rx_len > 0) try_parse_response(0xFE, rx, rx_len);
    usleep(200000);

    log_msg("Step 3: Get radar state\n");
    rx_len = send_and_recv(g_fd, 0xD0, NULL, 0, rx, sizeof(rx), 1000);
    if (g_verbose || rx_len > 0) {
        printf("  RX (%d bytes): ", rx_len);
        print_hex(rx, rx_len);
        printf("\n");
    }
    if (rx_len > 0) try_parse_response(0xD0, rx, rx_len);
    usleep(200000);

    log_msg("Step 4: Get algorithm type\n");
    rx_len = send_and_recv(g_fd, 0xD2, NULL, 0, rx, sizeof(rx), 1000);
    if (g_verbose || rx_len > 0) {
        printf("  RX (%d bytes): ", rx_len);
        print_hex(rx, rx_len);
        printf("\n");
    }
    if (rx_len > 0) try_parse_response(0xD2, rx, rx_len);
    usleep(200000);

    log_msg("Step 5: Enable radar detection\n");
    rx_len = send_and_recv(g_fd, 0xD1, (uint8_t[]){0x01}, 1, rx, sizeof(rx), 1000);
    if (g_verbose || rx_len > 0) {
        printf("  RX (%d bytes): ", rx_len);
        print_hex(rx, rx_len);
        printf("\n");
    }
    if (rx_len > 0) try_parse_response(0xD1, rx, rx_len);
    usleep(500000);

    log_msg("Step 6: Entering main loop - waiting for BSD reports...\n");
    log_msg("Note: BSD reports are only output when targets are detected.\n");
    log_msg("Move objects in front of the radar to trigger detection.\n");

    int loop_count = 0;
    while (g_running) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(g_fd, &rfds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select(g_fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        loop_count++;
        if (loop_count % 5 == 0 && g_verbose) {
            printf("[Waiting for data... %ds]\n", loop_count);
        }

        if (ret > 0 && FD_ISSET(g_fd, &rfds)) {
            rx_len = read(g_fd, rx, sizeof(rx));
            if (rx_len > 0) {
                printf("RX (%d bytes): ", rx_len);
                print_hex(rx, rx_len);
                printf("\n");

                if (rx[0] == FRAME_HEAD_REPORT) {
                    try_parse_report(rx, rx_len);
                } else if (rx[0] == FRAME_HEAD_RESP) {
                    try_parse_response(0, rx, rx_len);
                } else {
                    printf("  [Unknown header 0x%02X, trying both parsers]\n", rx[0]);
                    if (try_parse_report(rx, rx_len) != 0) {
                        try_parse_response(0, rx, rx_len);
                    }
                }
            }
        }
    }

    log_msg("Radar BSD v2 stopping\n");
    if (g_log_file) fclose(g_log_file);
    close(g_fd);
    return 0;
}
