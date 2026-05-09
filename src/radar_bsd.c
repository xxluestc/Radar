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
#define MAX_FRAME_SIZE 256
#define MAX_BSD_OBJECTS 8

#define FRAME_HEAD_CMD 0x58
#define FRAME_HEAD_RESP 0x59
#define FRAME_HEAD_REPORT 0x5A

#define REPORT_TYPE_BSD 7

#pragma pack(push, 1)
typedef struct {
    int8_t range_val;
    int8_t angle_val;
    int8_t velo_val;
    int8_t objId;
} bsd_obj_info_t;

typedef struct {
    uint16_t obj_num;
    uint16_t reserved;
    bsd_obj_info_t obj[MAX_BSD_OBJECTS];
} bsd_det_info_t;
#pragma pack(pop)

static volatile int g_running = 1;
static int g_fd = -1;
static int g_verbose = 0;
static int g_warning_enabled = 1;
static int g_warning_distance = 5;
static int g_warning_approach_speed = 2;
static FILE *g_log_file = NULL;

static void signal_handler(int sig)
{
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
        case 9600: speed = B9600; break;
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

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

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

static uint8_t calc_checksum(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
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

static void log_message(const char *fmt, ...)
{
    char ts[32];
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
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
}

static int send_cmd(int fd, uint8_t cmd_group, uint8_t cmd, const uint8_t *params, int param_len)
{
    uint8_t frame[64];
    int idx = 0;

    frame[idx++] = FRAME_HEAD_CMD;
    frame[idx++] = (cmd_group << 5) | cmd;
    frame[idx++] = (uint8_t)param_len;

    if (params && param_len > 0) {
        memcpy(&frame[idx], params, param_len);
        idx += param_len;
    }

    uint8_t checksum = calc_checksum(frame, idx);
    frame[idx++] = checksum;
    frame[idx++] = 0x00;

    int written = write(fd, frame, idx);
    if (written != idx) {
        fprintf(stderr, "write failed: %s\n", strerror(errno));
        return -1;
    }

    tcdrain(fd);

    if (g_verbose) {
        printf("TX: ");
        print_hex(frame, idx);
        printf("\n");
    }

    return 0;
}

static int send_bsd_enable_cmd(int fd)
{
    uint8_t params[2] = {0x01, 0x00};
    return send_cmd(fd, 0x06, 0x10, params, 2);
}

static int send_auto_report_enable_cmd(int fd)
{
    uint8_t params[2] = {0x01, 0x00};
    return send_cmd(fd, 0x06, 0x12, params, 2);
}

static int send_set_baudrate_cmd(int fd, uint32_t baudrate)
{
    uint8_t params[4];
    params[0] = baudrate & 0xFF;
    params[1] = (baudrate >> 8) & 0xFF;
    params[2] = (baudrate >> 16) & 0xFF;
    params[3] = (baudrate >> 24) & 0xFF;
    return send_cmd(fd, 0x00, 0x19, params, 4);
}

static void process_bsd_report(const bsd_det_info_t *bsd)
{
    int obj_count = bsd->obj_num;
    if (obj_count > MAX_BSD_OBJECTS) {
        obj_count = MAX_BSD_OBJECTS;
    }

    log_message("BSD Report: %d target(s) detected\n", obj_count);

    int danger_detected = 0;

    for (int i = 0; i < obj_count; i++) {
        const bsd_obj_info_t *obj = &bsd->obj[i];
        log_message("  Target[%d]: ID=%d, Distance=%dm, Angle=%d°, Speed=%dm/s\n",
                    i, obj->objId, obj->range_val, obj->angle_val, obj->velo_val);

        if (g_warning_enabled) {
            if (obj->range_val > 0 && obj->range_val <= g_warning_distance && obj->velo_val > g_warning_approach_speed) {
                danger_detected = 1;
                log_message("  ⚠️  WARNING: Target %d approaching! Distance=%dm, Speed=%dm/s\n",
                           obj->objId, obj->range_val, obj->velo_val);
            }
        }
    }

    if (g_warning_enabled && danger_detected) {
        log_message("🚨 DANGER ALERT: Approaching target detected from behind!\n");
    }
}

static int process_report_frame(const uint8_t *payload, int payload_len)
{
    if (payload_len < 1) {
        return -1;
    }

    uint8_t type = payload[0];
    const uint8_t *data = payload + 1;
    int data_len = payload_len - 1;

    if (g_verbose) {
        printf("Report TYPE=%d, data_len=%d\n", type, data_len);
    }

    switch (type) {
        case REPORT_TYPE_BSD: {
            if (data_len < (int)sizeof(uint16_t)) {
                if (g_verbose) fprintf(stderr, "BSD report too short: %d\n", data_len);
                return -1;
            }
            bsd_det_info_t bsd;
            memset(&bsd, 0, sizeof(bsd));
            bsd.obj_num = data[0] | (data[1] << 8);
            if (data_len >= 4) {
                bsd.reserved = data[2] | (data[3] << 8);
            }
            int obj_count = bsd.obj_num;
            if (obj_count > MAX_BSD_OBJECTS) obj_count = MAX_BSD_OBJECTS;
            int expected_len = 4 + obj_count * sizeof(bsd_obj_info_t);
            if (data_len < expected_len) {
                if (g_verbose) fprintf(stderr, "BSD data incomplete: expected %d, got %d\n", expected_len, data_len);
                obj_count = (data_len - 4) / sizeof(bsd_obj_info_t);
                if (obj_count < 0) obj_count = 0;
            }
            for (int i = 0; i < obj_count; i++) {
                int offset = 4 + i * sizeof(bsd_obj_info_t);
                if (offset + sizeof(bsd_obj_info_t) <= data_len) {
                    memcpy(&bsd.obj[i], &data[offset], sizeof(bsd_obj_info_t));
                }
            }
            process_bsd_report(&bsd);
            break;
        }
        default:
            if (g_verbose) {
                printf("Unhandled report type: %d\n", type);
            }
            break;
    }

    return 0;
}

static int process_response_frame(const uint8_t *payload, int payload_len)
{
    if (g_verbose) {
        printf("Response: ");
        print_hex(payload, payload_len);
        printf("\n");
    }

    if (payload_len >= 3) {
        uint8_t status = payload[2];
        if (status == 0x00) {
            if (g_verbose) printf("  Command executed successfully\n");
        } else {
            if (g_verbose) printf("  Command failed, status=0x%02X\n", status);
        }
    }

    return 0;
}

static int process_frame(const uint8_t *frame, int frame_len)
{
    if (frame_len < 3) return -1;

    uint8_t head = frame[0];
    uint8_t len = frame[1];

    if (head == FRAME_HEAD_REPORT) {
        if (g_verbose) {
            printf("RX Report: ");
            print_hex(frame, frame_len);
            printf("\n");
        }
        return process_report_frame(&frame[2], len);
    } else if (head == FRAME_HEAD_RESP) {
        if (g_verbose) {
            printf("RX Response: ");
            print_hex(frame, frame_len);
            printf("\n");
        }
        return process_response_frame(&frame[2], len);
    }

    return -1;
}

static int receive_and_process(int fd)
{
    static uint8_t rx_buf[512];
    static int rx_len = 0;

    int bytes_read = read(fd, rx_buf + rx_len, sizeof(rx_buf) - rx_len - 1);
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        fprintf(stderr, "read error: %s\n", strerror(errno));
        return -1;
    }
    if (bytes_read == 0) {
        return 0;
    }

    rx_len += bytes_read;

    while (rx_len >= 3) {
        uint8_t head = rx_buf[0];

        if (head != FRAME_HEAD_REPORT && head != FRAME_HEAD_RESP && head != FRAME_HEAD_CMD) {
            int i;
            for (i = 1; i < rx_len; i++) {
                if (rx_buf[i] == FRAME_HEAD_REPORT || rx_buf[i] == FRAME_HEAD_RESP || rx_buf[i] == FRAME_HEAD_CMD) {
                    break;
                }
            }
            if (g_verbose && i > 1) {
                printf("Skipped %d invalid bytes\n", i);
            }
            memmove(rx_buf, rx_buf + i, rx_len - i);
            rx_len -= i;
            continue;
        }

        uint8_t len = rx_buf[1];
        int total_frame_len = 2 + len + 1;

        if (total_frame_len > rx_len) {
            break;
        }

        uint8_t checksum = calc_checksum(rx_buf, 2 + len);
        if (checksum != rx_buf[2 + len]) {
            if (g_verbose) {
                printf("Checksum mismatch: calc=0x%02X, recv=0x%02X\n", checksum, rx_buf[2 + len]);
            }
            memmove(rx_buf, rx_buf + 1, rx_len - 1);
            rx_len--;
            continue;
        }

        process_frame(rx_buf, total_frame_len);

        memmove(rx_buf, rx_buf + total_frame_len, rx_len - total_frame_len);
        rx_len -= total_frame_len;
    }

    if (rx_len >= (int)sizeof(rx_buf) - 1) {
        rx_len = 0;
    }

    return 0;
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("Options:\n");
    printf("  -d DEVICE   UART device (default: %s)\n", DEFAULT_UART_DEVICE);
    printf("  -b BAUD     Baud rate (default: %d)\n", DEFAULT_BAUDRATE);
    printf("  -v          Verbose output\n");
    printf("  -l FILE     Log to file\n");
    printf("  -W DIST     Warning distance in meters (default: %d)\n", g_warning_distance);
    printf("  -S SPEED    Warning approach speed in m/s (default: %d)\n", g_warning_approach_speed);
    printf("  -n          Disable warning alerts\n");
    printf("  -h          Show this help\n");
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
            case 'h': print_usage(argv[0]); return 0;
            default: print_usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (logfile) {
        g_log_file = fopen(logfile, "a");
        if (!g_log_file) {
            fprintf(stderr, "Cannot open log file: %s\n", logfile);
            return 1;
        }
    }

    log_message("Radar BSD Detection Program starting\n");
    log_message("Device: %s, Baudrate: %d\n", device, baudrate);
    log_message("Warning: distance<=%dm, approach_speed>%dm/s, enabled=%d\n",
               g_warning_distance, g_warning_approach_speed, g_warning_enabled);

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

    log_message("UART configured successfully\n");

    usleep(100000);

    log_message("Enabling BSD detection...\n");
    send_bsd_enable_cmd(g_fd);
    usleep(50000);

    log_message("Enabling auto report...\n");
    send_auto_report_enable_cmd(g_fd);
    usleep(50000);

    log_message("Waiting for radar data...\n");

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
            fprintf(stderr, "select error: %s\n", strerror(errno));
            break;
        }

        if (ret > 0 && FD_ISSET(g_fd, &rfds)) {
            receive_and_process(g_fd);
        }
    }

    log_message("Radar BSD Detection Program stopping\n");

    if (g_log_file) {
        fclose(g_log_file);
    }
    close(g_fd);

    return 0;
}
