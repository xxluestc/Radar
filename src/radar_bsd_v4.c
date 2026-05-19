#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>

static volatile int g_running = 1;
static int g_verbose = 0;
static int g_fd = -1;
static FILE *g_log_file = NULL;
static int g_radar_enabled = 0;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

static void log_msg(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char ts[32];
    strftime(ts, sizeof(ts), "[%Y-%m-%d %H:%M:%S]", tm);

    printf("%s %s", ts, buf);
    fflush(stdout);

    if (g_log_file) {
        fprintf(g_log_file, "%s %s", ts, buf);
        fflush(g_log_file);
    }
}

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
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

static int read_raw(int fd, uint8_t *buf, int max_len, int timeout_ms)
{
    int total = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (total < max_len - 1) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 5000;
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, max_len - total);
            if (n > 0) total += n;
        }
        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                      (now.tv_usec - start.tv_usec) / 1000;
        if (total > 0 && elapsed >= timeout_ms) break;
        if (elapsed >= timeout_ms * 2) break;
    }
    return total;
}

static int find_response_frame(const uint8_t *raw, int raw_len,
                                uint8_t *frame, int max_frame,
                                uint8_t expected_cmd, uint8_t expected_plen)
{
    for (int off = 0; off < raw_len - 4; off++) {
        if (raw[off] != 0x59) continue;
        if (off + 2 >= raw_len) continue;
        int plen = raw[off + 2];
        int frame_len = 3 + plen + 2;
        if (off + frame_len > raw_len) continue;
        if (raw[off + 1] != expected_cmd && expected_cmd != 0) continue;

        uint16_t ck_calc = calc_checksum16(raw + off, 3 + plen);
        uint16_t ck_recv = raw[off + 3 + plen] | (raw[off + 3 + plen + 1] << 8);
        if (ck_calc == ck_recv) {
            if (frame_len <= max_frame) {
                memcpy(frame, raw + off, frame_len);
                return frame_len;
            }
        }
    }

    if (expected_cmd == 0 || expected_plen == 0) return -1;

    for (int off = 0; off < raw_len - 2; off++) {
        int tail_len = raw_len - off;
        if (tail_len < 2) continue;

        int payload_len = tail_len - 2;
        if (payload_len != expected_plen) continue;

        uint16_t ck_recv = raw[off + payload_len] | (raw[off + payload_len + 1] << 8);

        uint8_t reconstructed[256];
        reconstructed[0] = 0x59;
        reconstructed[1] = expected_cmd;
        reconstructed[2] = expected_plen;
        memcpy(reconstructed + 3, raw + off, payload_len);

        uint16_t ck_calc = calc_checksum16(reconstructed, 3 + payload_len);
        if (ck_calc == ck_recv) {
            int total = 3 + payload_len + 2;
            if (total <= max_frame) {
                memcpy(frame, reconstructed, 3 + payload_len);
                frame[3 + payload_len] = raw[off + payload_len];
                frame[3 + payload_len + 1] = raw[off + payload_len + 1];
                return total;
            }
        }
    }

    return -1;
}

static int find_report_frame(const uint8_t *raw, int raw_len,
                              uint8_t *frame, int max_frame)
{
    for (int off = 0; off < raw_len - 4; off++) {
        if (raw[off] != 0x5A) continue;
        if (off + 2 >= raw_len) continue;
        int plen = raw[off + 2];
        int frame_len = 3 + plen + 1;
        if (off + frame_len > raw_len) continue;

        uint8_t ck_calc = calc_checksum8(raw + off, 3 + plen);
        uint8_t ck_recv = raw[off + 3 + plen];
        if (ck_calc == ck_recv) {
            if (frame_len <= max_frame) {
                memcpy(frame, raw + off, frame_len);
                return frame_len;
            }
        }
    }
    return -1;
}

static int send_and_read(int fd, uint8_t cmd_byte, const uint8_t *params, int param_len,
                          uint8_t *response, int max_resp, int timeout_ms,
                          uint8_t expected_plen)
{
    uint8_t tmp[256];
    while (read(fd, tmp, sizeof(tmp)) > 0) {}
    usleep(50000);

    int cmd_len = 3 + param_len + 2;
    uint8_t cmd[64];
    cmd[0] = 0x58;
    cmd[1] = cmd_byte;
    cmd[2] = param_len;
    if (params && param_len > 0) memcpy(cmd + 3, params, param_len);
    uint16_t ck = calc_checksum16(cmd, 3 + param_len);
    cmd[3 + param_len] = ck & 0xFF;
    cmd[3 + param_len + 1] = (ck >> 8) & 0xFF;

    if (g_verbose) {
        log_msg("TX: ");
        char hex[256];
        int pos = 0;
        for (int i = 0; i < cmd_len; i++) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", cmd[i]);
        log_msg("%s\n", hex);
    }

    write(fd, cmd, cmd_len);
    tcdrain(fd);

    uint8_t raw[1024];
    int raw_len = read_raw(fd, raw, sizeof(raw), timeout_ms);

    if (raw_len <= 0) return -1;

    if (g_verbose) {
        log_msg("RX (%d bytes): ", raw_len);
        char hex[256];
        int pos = 0;
        for (int i = 0; i < raw_len && i < 32; i++) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", raw[i]);
        if (raw_len > 32) pos += snprintf(hex + pos, sizeof(hex) - pos, "...");
        log_msg("%s\n", hex);
    }

    int frame_len = find_response_frame(raw, raw_len, response, max_resp, cmd_byte, expected_plen);
    if (frame_len > 0) return frame_len;

    frame_len = find_response_frame(raw, raw_len, response, max_resp, 0, 0);
    if (frame_len > 0) return frame_len;

    return -1;
}

static int send_cmd_retry(int fd, uint8_t cmd_byte, const uint8_t *params, int param_len,
                           uint8_t *response, int max_resp, int max_retries, int timeout_ms,
                           uint8_t expected_plen)
{
    for (int i = 0; i < max_retries && g_running; i++) {
        int result = send_and_read(fd, cmd_byte, params, param_len, response, max_resp, timeout_ms, expected_plen);
        if (result > 0) return result;

        if (i < max_retries - 1) {
            if (g_verbose) log_msg("Retry %d/%d...\n", i + 2, max_retries);
            usleep(100000);
        }
    }
    return -1;
}

static int parse_bsd_report(const uint8_t *frame, int frame_len)
{
    if (frame_len < 5) return -1;
    if (frame[0] != 0x5A) return -1;

    int plen = frame[2];
    if (frame_len < 3 + plen + 1) return -1;

    uint8_t ck_calc = calc_checksum8(frame, 3 + plen);
    uint8_t ck_recv = frame[3 + plen];
    if (ck_calc != ck_recv) {
        if (g_verbose) log_msg("BSD checksum mismatch: calc=%02X recv=%02X\n", ck_calc, ck_recv);
        return -1;
    }

    int type = frame[1];
    if (type != 7) {
        if (g_verbose) log_msg("BSD type=%d (not target report)\n", type);
        return 0;
    }

    int payload_len = plen;
    const uint8_t *payload = frame + 3;

    if (payload_len < 4) return -1;
    int obj_num = payload[0] | (payload[1] << 8);

    if (obj_num == 0) {
        log_msg("No targets detected\n");
        return 0;
    }

    log_msg("Targets: %d\n", obj_num);

    for (int i = 0; i < obj_num && i < 10; i++) {
        int off = 4 + i * 4;
        if (off + 4 > payload_len) break;

        uint8_t dist = payload[off];
        uint8_t speed = payload[off + 1];
        uint8_t angle = payload[off + 2];
        uint8_t rsv = payload[off + 3];

        log_msg("  T%d: dist=%d speed=%d angle=%d rsv=%d\n",
                i + 1, dist, speed, angle, rsv);
    }

    return obj_num;
}

int main(int argc, char *argv[])
{
    const char *device = "/dev/ttySTM3";
    int baudrate = 921600;
    const char *logfile = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) g_verbose = 1;
        else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) device = argv[++i];
        else if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) baudrate = atoi(argv[++i]);
        else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc) logfile = argv[++i];
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (logfile) {
        g_log_file = fopen(logfile, "a");
        if (!g_log_file) fprintf(stderr, "Cannot open log file: %s\n", logfile);
    }

    log_msg("Radar BSD v4 - Frame Repair\n");
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
    uint8_t resp[256];
    int len = send_cmd_retry(g_fd, 0xFE, NULL, 0, resp, sizeof(resp), 10, 500, 0x08);
    if (len > 0) {
        log_msg("Version response (%d bytes): ", len);
        char hex[256];
        int pos = 0;
        for (int i = 0; i < len; i++) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", resp[i]);
        log_msg("%s\n", hex);
        if (len >= 13) {
            log_msg("  SW: v%d.%d.%d, HW: v%d.%d\n",
                    resp[3], resp[4], resp[5], resp[8], resp[9]);
        }
    } else {
        log_msg("Failed to get version\n");
    }
    usleep(100000);

    log_msg("Step 2: Get radar state\n");
    len = send_cmd_retry(g_fd, 0xD0, NULL, 0, resp, sizeof(resp), 10, 500, 0x01);
    if (len > 0) {
        int state = (len >= 5) ? resp[3] : -1;
        log_msg("Radar state: %s\n", state == 1 ? "ON" : (state == 0 ? "OFF" : "UNKNOWN"));
        g_radar_enabled = (state == 1);
    } else {
        log_msg("Failed to get radar state\n");
    }
    usleep(100000);

    if (!g_radar_enabled) {
        log_msg("Step 3: Enable radar\n");
        uint8_t param = 0x01;
        len = send_cmd_retry(g_fd, 0xD1, &param, 1, resp, sizeof(resp), 10, 500, 0x01);
        if (len > 0) {
            log_msg("Radar enabled\n");
            g_radar_enabled = 1;
        } else {
            log_msg("Failed to enable radar\n");
        }
        usleep(100000);
    }

    log_msg("Step 4: Reading BSD reports...\n");
    log_msg("Press Ctrl+C to stop\n");

    while (g_running) {
        uint8_t raw[4096];
        int raw_len = read_raw(g_fd, raw, sizeof(raw), 200);

        if (raw_len > 0) {
            uint8_t frame[1024];
            int frame_len = find_report_frame(raw, raw_len, frame, sizeof(frame));
            if (frame_len > 0) {
                parse_bsd_report(frame, frame_len);
            } else if (g_verbose) {
                log_msg("Raw (%d bytes): ", raw_len);
                char hex[256];
                int pos = 0;
                for (int i = 0; i < raw_len && i < 32; i++) pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", raw[i]);
                if (raw_len > 32) pos += snprintf(hex + pos, sizeof(hex) - pos, "...");
                log_msg("%s\n", hex);
            }
        }
    }

    log_msg("Shutting down...\n");

    if (g_radar_enabled) {
        uint8_t param = 0x00;
        uint8_t resp2[256];
        send_cmd_retry(g_fd, 0xD1, &param, 1, resp2, sizeof(resp2), 3, 500, 0x01);
        log_msg("Radar disabled\n");
    }

    close(g_fd);
    if (g_log_file) fclose(g_log_file);
    log_msg("Done\n");
    return 0;
}