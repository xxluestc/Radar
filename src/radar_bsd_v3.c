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

#define DEFAULT_UART_DEVICE "/dev/ttySTM3"
#define DEFAULT_BAUDRATE 921600

#define FRAME_HEAD_CMD    0x58
#define FRAME_HEAD_RESP   0x59
#define FRAME_HEAD_REPORT 0x5A
#define REPORT_TYPE_BSD   7
#define MAX_BSD_OBJECTS   8

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

static uint16_t calc_checksum16(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

static uint8_t calc_checksum8(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint8_t)(sum & 0xFF);
}

static int read_and_parse(int fd, int timeout_ms);

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
    uint16_t checksum = calc_checksum16(frame, idx);
    frame[idx++] = (uint8_t)(checksum & 0xFF);
    frame[idx++] = (uint8_t)((checksum >> 8) & 0xFF);

    if (g_verbose) {
        printf("TX: ");
        print_hex(frame, idx);
        printf("\n");
    }

    write(fd, frame, idx);
    tcdrain(fd);
    return 0;
}

static int send_cmd_retry(int fd, uint8_t cmd_byte, const uint8_t *params, int param_len,
                          int max_retries, int timeout_ms)
{
    int i;
    for (i = 0; i < max_retries && g_running; i++) {
        {
            uint8_t tmp[256];
            while (read(fd, tmp, sizeof(tmp)) > 0) {}
        }
        usleep(50000);

        send_cmd(fd, cmd_byte, params, param_len);

        int result = read_and_parse(fd, timeout_ms);
        if (result > 0) return result;

        if (i < max_retries - 1) {
            if (g_verbose) log_msg("Retry %d/%d...\n", i + 2, max_retries);
            usleep(100000);
        }
    }
    return -1;
}

static int parse_response_frame(const uint8_t *data, int data_len)
{
    if (data_len < 5) return -1;
    if (data[0] != FRAME_HEAD_RESP) return -1;

    uint8_t cmd = data[1];
    uint8_t plen = data[2];
    int expected = 3 + plen + 2;
    if (data_len < expected) return -1;

    uint16_t recv_cksum = data[3 + plen] | (data[3 + plen + 1] << 8);
    uint16_t calc_cksum = calc_checksum16(data, 3 + plen);

    if (recv_cksum != calc_cksum) {
        if (g_verbose)
            printf("  [Response checksum mismatch: recv=0x%04X calc=0x%04X]\n",
                   recv_cksum, calc_cksum);
        return -1;
    }

    if (g_verbose) {
        printf("  [Response: cmd=0x%02X, plen=%d] ", cmd, plen);
        print_hex(data, expected);
        printf("\n");
    }

    switch (cmd) {
        case 0xFE:
            if (plen >= 8)
                log_msg("Version: SW=%d.%d.%d, Customer=%d.%d, HW=%d.%d, Reserved=%d\n",
                       data[3], data[4], data[5], data[6], data[7],
                       data[8], data[9], data[10]);
            break;
        case 0xD0:
            if (plen >= 1)
                log_msg("Radar state: %s (0x%02X)\n",
                       data[3] ? "ON" : "OFF", data[3]);
            break;
        case 0xD1:
            if (plen >= 1)
                log_msg("Radar enable: %s (0x%02X)\n",
                       data[3] == 0x00 ? "SUCCESS" : "FAILED", data[3]);
            break;
        case 0xD2:
            if (plen >= 1)
                log_msg("Algorithm type: %d\n", data[3]);
            break;
        case 0x90:
            if (plen >= 1)
                log_msg("ULP active time: %s (0x%02X)\n",
                       data[3] == 0x00 ? "SUCCESS" : "FAILED", data[3]);
            break;
        default:
            log_msg("Unknown response cmd=0x%02X, plen=%d\n", cmd, plen);
            break;
    }
    return expected;
}

static int parse_report_frame(const uint8_t *data, int data_len)
{
    if (data_len < 4) return -1;
    if (data[0] != FRAME_HEAD_REPORT) return -1;

    uint8_t plen = data[1];
    int expected = 2 + plen + 1;
    if (data_len < expected) return -1;

    uint8_t recv_ck = data[2 + plen];
    uint8_t calc_ck = calc_checksum8(data, 2 + plen);

    if (recv_ck != calc_ck) {
        if (g_verbose)
            printf("  [Report checksum mismatch: recv=0x%02X calc=0x%02X]\n",
                   recv_ck, calc_ck);
        return -1;
    }

    uint8_t rtype = data[2];
    int payload_len = plen - 1;

    if (g_verbose) {
        printf("  [Report: type=%d, plen=%d] ", rtype, plen);
        print_hex(data, expected);
        printf("\n");
    }

    if (rtype == REPORT_TYPE_BSD && payload_len >= 4) {
        int obj_num = data[3] | (data[4] << 8);
        if (obj_num > MAX_BSD_OBJECTS) obj_num = MAX_BSD_OBJECTS;

        int avail_objs = (payload_len - 4) / 4;
        if (obj_num > avail_objs) obj_num = avail_objs;

        log_msg("BSD Report: %d target(s)\n", obj_num);
        int danger = 0;
        for (int i = 0; i < obj_num; i++) {
            int off = 7 + i * 4;
            if (off + 4 > 2 + plen) break;
            int8_t dist = (int8_t)data[off];
            int8_t angle = (int8_t)data[off + 1];
            int8_t speed = (int8_t)data[off + 2];
            int8_t id = (int8_t)data[off + 3];
            log_msg("  Target[%d]: ID=%d, Dist=%dm, Angle=%d, Speed=%dm/s\n",
                   i, id, dist, angle, speed);
            if (g_warning_enabled && dist > 0 && dist <= g_warning_distance &&
                speed > g_warning_approach_speed) {
                danger = 1;
                log_msg("  WARNING: Target approaching! Dist=%dm, Speed=%dm/s\n", dist, speed);
            }
        }
        if (danger) log_msg("DANGER ALERT!\n");
    }
    return expected;
}

static int read_and_parse(int fd, int timeout_ms)
{
    uint8_t buf[1024];
    int total = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (g_running && total < (int)sizeof(buf)) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 5000;

        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, sizeof(buf) - total);
            if (n > 0) total += n;
        }

        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                      (now.tv_usec - start.tv_usec) / 1000;
        if (total > 0 && elapsed >= timeout_ms) break;
        if (elapsed >= timeout_ms + 500) break;
    }

    if (total == 0) return 0;

    if (g_verbose) {
        printf("RX (%d bytes): ", total);
        print_hex(buf, total);
        printf("\n");
    }

    int offset = 0;
    int parsed_any = 0;
    while (offset < total) {
        uint8_t head = buf[offset];
        int remaining = total - offset;
        int consumed = 0;

        if (head == FRAME_HEAD_RESP && remaining >= 5) {
            consumed = parse_response_frame(buf + offset, remaining);
        } else if (head == FRAME_HEAD_REPORT && remaining >= 4) {
            consumed = parse_report_frame(buf + offset, remaining);
        }

        if (consumed > 0) {
            offset += consumed;
            parsed_any = 1;
        } else {
            if (g_verbose && head != 0x00) {
                printf("  [Skipping byte 0x%02X at offset %d]\n", head, offset);
            }
            offset++;
        }
    }

    return parsed_any ? total : -1;
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

    log_msg("Radar BSD v3 - Fixed timing\n");
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

    {
        uint8_t tmp[256];
        while (read(g_fd, tmp, sizeof(tmp)) > 0) {}
    }
    usleep(100000);

    log_msg("Step 1: Get version\n");
    send_cmd_retry(g_fd, 0xFE, NULL, 0, 10, 500);
    usleep(100000);

    log_msg("Step 2: Get radar state\n");
    send_cmd_retry(g_fd, 0xD0, NULL, 0, 10, 500);
    usleep(100000);

    log_msg("Step 3: Set ULP active time to 255s (prevent sleep)\n");
    send_cmd_retry(g_fd, 0x90, (uint8_t[]){0xFF}, 1, 10, 500);
    usleep(100000);

    log_msg("Step 4: Enable radar detection\n");
    send_cmd_retry(g_fd, 0xD1, (uint8_t[]){0x01}, 1, 10, 500);
    usleep(100000);

    log_msg("Step 5: Verify radar state\n");
    send_cmd_retry(g_fd, 0xD0, NULL, 0, 10, 500);
    usleep(500000);

    log_msg("Step 6: Entering main loop - waiting for BSD reports...\n");
    log_msg("BSD reports (TYPE=7, HEAD=0x5A) are sent when targets detected.\n");
    log_msg("Move objects in front of radar to trigger detection.\n");

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
        if (loop_count % 5 == 0) {
            printf("[Waiting for data... %ds]\n", loop_count);
        }

        if (ret > 0 && FD_ISSET(g_fd, &rfds)) {
            read_and_parse(g_fd, 100);
        }
    }

    log_msg("Radar BSD v3 stopping\n");
    if (g_log_file) fclose(g_log_file);
    close(g_fd);
    return 0;
}