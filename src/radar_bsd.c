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
#define MAX_FRAME_SIZE 512
#define MAX_BSD_OBJECTS 8

#define FRAME_HEAD_CMD 0x58
#define FRAME_HEAD_RESP 0x59
#define FRAME_HEAD_REPORT 0x5A

#define REPORT_TYPE_BSD 7

#include <linux/gpio.h>

#define GPIO_CHIP "/dev/gpiochip1"
#define GPIO_OUT_PIN 2

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
static int g_gpio_fd = -1;
static int g_verbose = 0;
static int g_warning_enabled = 1;
static int g_warning_distance = 5;
static int g_warning_approach_speed = 2;
static FILE *g_log_file = NULL;
static uint8_t g_last_sent_cmd = 0;
static int g_header_loss_detected = 0;

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

static uint16_t calc_checksum(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

static uint8_t calc_report_checksum(const uint8_t *data, int len)
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

    int written = write(fd, frame, idx);
    if (written != idx) {
        fprintf(stderr, "write failed: %s\n", strerror(errno));
        return -1;
    }

    tcdrain(fd);

    g_last_sent_cmd = cmd_byte;

    if (g_verbose) {
        printf("TX: ");
        print_hex(frame, idx);
        printf("\n");
    }

    return 0;
}

static int send_system_reset_cmd(int fd)
{
    uint8_t param = 0x01;
    return send_cmd(fd, 0x13, &param, 1);
}

static int send_get_version_cmd(int fd)
{
    return send_cmd(fd, 0xFE, NULL, 0);
}

static int send_radar_enable_cmd(int fd)
{
    uint8_t param = 0x01;
    return send_cmd(fd, 0xD1, &param, 1);
}

static int send_radar_disable_cmd(int fd)
{
    uint8_t param = 0x00;
    return send_cmd(fd, 0xD1, &param, 1);
}

static int send_get_radar_state_cmd(int fd)
{
    return send_cmd(fd, 0xD0, NULL, 0);
}

static int send_get_algo_type_cmd(int fd)
{
    return send_cmd(fd, 0xD2, NULL, 0);
}

static int send_set_ulp_active_time_cmd(int fd, uint8_t seconds)
{
    return send_cmd(fd, 0x90, &seconds, 1);
}

static int init_gpio_out(void)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", GPIO_OUT_PIN + 32);
    g_gpio_fd = open(path, O_RDONLY);
    if (g_gpio_fd < 0) {
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", GPIO_OUT_PIN + 32 * 1);
        g_gpio_fd = open(path, O_RDONLY);
    }
    if (g_gpio_fd < 0) {
        fprintf(stderr, "Cannot open GPIO OUT sysfs: %s\n", strerror(errno));
        return -1;
    }
    return g_gpio_fd;
}

static int read_gpio_out(int line_fd)
{
    char buf[4] = {0};
    lseek(line_fd, 0, SEEK_SET);
    if (read(line_fd, buf, sizeof(buf) - 1) < 1) {
        return -1;
    }
    return atoi(buf);
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
        log_message("  Target[%d]: ID=%d, Distance=%dm, Angle=%d, Speed=%dm/s\n",
                    i, obj->objId, obj->range_val, obj->angle_val, obj->velo_val);

        if (g_warning_enabled) {
            if (obj->range_val > 0 && obj->range_val <= g_warning_distance && obj->velo_val > g_warning_approach_speed) {
                danger_detected = 1;
                log_message("  WARNING: Target %d approaching! Distance=%dm, Speed=%dm/s\n",
                           obj->objId, obj->range_val, obj->velo_val);
            }
        }
    }

    if (g_warning_enabled && danger_detected) {
        log_message("DANGER ALERT: Approaching target detected from behind!\n");
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
        printf("RX Response payload: ");
        print_hex(payload, payload_len);
        printf("\n");
    }

    if (payload_len < 3) {
        if (g_verbose) printf("Response too short: %d bytes\n", payload_len);
        return -1;
    }

    uint8_t cmd_byte = payload[0];
    uint8_t param_len = payload[1];
    const uint8_t *params = &payload[2];

    if (2 + param_len > payload_len) {
        if (g_verbose) printf("Response param_len=%d exceeds payload\n", param_len);
        return -1;
    }

    if (cmd_byte == 0xFE) {
        if (param_len >= 8) {
            log_message("Version: SW=%d.%d.%d, Customer=%d.%d, CI=%d.%d, Algo=%d\n",
                       params[0], params[1], params[2],
                       params[3], params[4],
                       params[5], params[6],
                       params[7]);
        } else {
            log_message("Version response (short): ");
            print_hex(params, param_len);
            printf("\n");
        }
    } else if (cmd_byte == 0xD0) {
        if (param_len >= 1) {
            log_message("Radar state: %s\n", params[0] ? "ON" : "OFF");
        }
    } else if (cmd_byte == 0xD2) {
        if (param_len >= 1) {
            log_message("Algorithm type: %d\n", params[0]);
        }
    } else if (cmd_byte == 0xD1) {
        if (param_len >= 1) {
            log_message("Radar enable: %s\n", params[0] == 0x00 ? "SUCCESS" : "FAILED");
        }
    } else if (cmd_byte == 0x13) {
        if (param_len >= 1) {
            log_message("System Reset: %s\n", params[0] == 0x00 ? "SUCCESS" : "FAILED");
        }
    } else if (cmd_byte == 0x90) {
        if (param_len >= 1) {
            log_message("ULP active time set: %s\n", params[0] == 0x00 ? "SUCCESS" : "FAILED");
        }
    } else {
        if (g_verbose) {
            printf("Response CMD=0x%02X, param_len=%d: ", cmd_byte, param_len);
            print_hex(params, param_len);
            printf("\n");
        }
    }

    return 0;
}

static int process_frame(const uint8_t *frame, int frame_len)
{
    if (frame_len < 4) return -1;

    uint8_t head = frame[0];

    if (head == FRAME_HEAD_REPORT) {
        if (g_verbose) {
            printf("RX Report frame: ");
            print_hex(frame, frame_len);
            printf("\n");
        }
        uint8_t len = frame[1];
        if (2 + len + 1 > frame_len) return -1;
        uint8_t check = calc_report_checksum(frame, 2 + len);
        if (check != frame[2 + len]) {
            if (g_verbose) printf("Report checksum mismatch: calc=0x%02X, recv=0x%02X\n", check, frame[2 + len]);
            return -1;
        }
        return process_report_frame(&frame[2], len);
    } else if (head == FRAME_HEAD_RESP) {
        if (g_verbose) {
            printf("RX Response frame: ");
            print_hex(frame, frame_len);
            printf("\n");
        }
        if (frame_len < 3) return -1;
        uint8_t cmd_byte = frame[1];
        uint8_t param_len = frame[2];
        if (3 + param_len + 2 > frame_len) return -1;
        uint16_t recv_checksum = frame[3 + param_len] | (frame[3 + param_len + 1] << 8);
        uint16_t calc_check = calc_checksum(frame, 3 + param_len);
        if (calc_check != recv_checksum) {
            if (g_verbose) printf("Response checksum mismatch: calc=0x%04X, recv=0x%04X\n", calc_check, recv_checksum);
            return -1;
        }
        uint8_t payload[256];
        payload[0] = cmd_byte;
        payload[1] = param_len;
        if (param_len > 0) {
            memcpy(&payload[2], &frame[3], param_len);
        }
        return process_response_frame(payload, 2 + param_len);
    }

    return -1;
}

static uint8_t g_rx_buf[512];
static int g_rx_len = 0;

static int receive_and_process(int fd)
{

    int bytes_read = read(fd, g_rx_buf + g_rx_len, sizeof(g_rx_buf) - g_rx_len - 1);
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

    if (g_verbose) {
        printf("RX raw (%d bytes): ", bytes_read);
        print_hex(g_rx_buf + g_rx_len, bytes_read);
        printf("\n");
    }

    g_rx_len += bytes_read;

    while (g_rx_len >= 4) {
        uint8_t head = g_rx_buf[0];

        if (head != FRAME_HEAD_REPORT && head != FRAME_HEAD_RESP && head != FRAME_HEAD_CMD) {
            if (g_last_sent_cmd != 0 && g_rx_len >= 3) {
                uint8_t repaired[512];
                repaired[0] = FRAME_HEAD_RESP;
                repaired[1] = g_last_sent_cmd;
                int repaired_len = 2 + g_rx_len;
                if (repaired_len > (int)sizeof(repaired)) repaired_len = sizeof(repaired);
                memcpy(&repaired[2], g_rx_buf, g_rx_len);

                if (repaired_len >= 5) {
                    uint8_t param_len = repaired[2];
                    int expected_frame_len = 3 + param_len + 2;
                    if (expected_frame_len == repaired_len) {
                        uint16_t recv_checksum = repaired[3 + param_len] | (repaired[3 + param_len + 1] << 8);
                        uint16_t calc_check = calc_checksum(repaired, 3 + param_len);
                        if (calc_check == recv_checksum) {
                            if (!g_header_loss_detected) {
                                g_header_loss_detected = 1;
                                log_message("Header loss compensation enabled (CH342F interference detected)\n");
                            }
                            if (g_verbose) {
                                printf("RX repaired (lost 0x59+CMD): ");
                                print_hex(repaired, repaired_len);
                                printf("\n");
                            }
                            process_frame(repaired, repaired_len);
                            g_rx_len = 0;
                            g_last_sent_cmd = 0;
                            continue;
                        }
                    }
                }

                if (g_last_sent_cmd != 0 && g_rx_len >= 2) {
                    repaired[0] = FRAME_HEAD_RESP;
                    int repaired_len2 = 1 + g_rx_len;
                    if (repaired_len2 > (int)sizeof(repaired)) repaired_len2 = sizeof(repaired);
                    memcpy(&repaired[1], g_rx_buf, g_rx_len);

                    if (repaired_len2 >= 5) {
                        uint8_t param_len2 = repaired[2];
                        int expected_frame_len2 = 3 + param_len2 + 2;
                        if (expected_frame_len2 == repaired_len2) {
                            uint16_t recv_checksum2 = repaired[3 + param_len2] | (repaired[3 + param_len2 + 1] << 8);
                            uint16_t calc_check2 = calc_checksum(repaired, 3 + param_len2);
                            if (calc_check2 == recv_checksum2) {
                                if (!g_header_loss_detected) {
                                    g_header_loss_detected = 1;
                                    log_message("Header loss compensation enabled (lost 0x59 only)\n");
                                }
                                if (g_verbose) {
                                    printf("RX repaired (lost 0x59): ");
                                    print_hex(repaired, repaired_len2);
                                    printf("\n");
                                }
                                process_frame(repaired, repaired_len2);
                                g_rx_len = 0;
                                g_last_sent_cmd = 0;
                                continue;
                            }
                        }
                    }
                }
            }

            int i;
            for (i = 1; i < g_rx_len; i++) {
                if (g_rx_buf[i] == FRAME_HEAD_REPORT || g_rx_buf[i] == FRAME_HEAD_RESP || g_rx_buf[i] == FRAME_HEAD_CMD) {
                    break;
                }
            }
            if (g_verbose && i > 1) {
                printf("Skipped %d invalid bytes: ", i);
                print_hex(g_rx_buf, i);
                printf("\n");
            }
            memmove(g_rx_buf, g_rx_buf + i, g_rx_len - i);
            g_rx_len -= i;
            if (g_rx_len == 0) break;
            continue;
        }

        if (g_rx_len < 2) break;

        int total_frame_len;
        if (head == FRAME_HEAD_REPORT) {
            uint8_t len = g_rx_buf[1];
            total_frame_len = 2 + len + 1;
        } else if (head == FRAME_HEAD_RESP) {
            if (g_rx_len < 3) break;
            uint8_t param_len = g_rx_buf[2];
            total_frame_len = 3 + param_len + 2;
        } else if (head == FRAME_HEAD_CMD) {
            if (g_rx_len < 3) break;
            uint8_t param_len = g_rx_buf[2];
            total_frame_len = 3 + param_len + 2;
        } else {
            memmove(g_rx_buf, g_rx_buf + 1, g_rx_len - 1);
            g_rx_len -= 1;
            continue;
        }

        if (total_frame_len > (int)sizeof(g_rx_buf)) {
            if (g_verbose) printf("Invalid frame length %d, discarding first byte\n", total_frame_len);
            memmove(g_rx_buf, g_rx_buf + 1, g_rx_len - 1);
            g_rx_len -= 1;
            continue;
        }

        if (total_frame_len > g_rx_len) {
            break;
        }

        process_frame(g_rx_buf, total_frame_len);

        memmove(g_rx_buf, g_rx_buf + total_frame_len, g_rx_len - total_frame_len);
        g_rx_len -= total_frame_len;
    }

    if (g_rx_len >= (int)sizeof(g_rx_buf) - 1) {
        g_rx_len = 0;
    }

    return 0;
}

static int send_cmd_wait_response(int fd, uint8_t cmd_byte, const uint8_t *params, int param_len, int timeout_ms)
{
    {
        uint8_t tmp[256];
        while (read(fd, tmp, sizeof(tmp)) > 0) { }
    }

    g_rx_len = 0;

    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    select(fd + 1, &rfds, NULL, NULL, &tv);

    if (send_cmd(fd, cmd_byte, params, param_len) != 0) {
        return -1;
    }

    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (g_running) {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 5000;

        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            receive_and_process(fd);
        }

        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;
        if (elapsed >= timeout_ms) {
            break;
        }
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

    int gpio_line_fd = init_gpio_out();
    if (gpio_line_fd >= 0) {
        int val = read_gpio_out(gpio_line_fd);
        log_message("Radar OUT (PB2) initial state: %d\n", val);
    } else {
        log_message("GPIO OUT pin not available, continuing without GPIO\n");
    }

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

    log_message("Step 1: Flushing any stale data...\n");
    {
        uint8_t tmp[256];
        int total_flushed = 0;
        while (1) {
            int n = read(g_fd, tmp, sizeof(tmp));
            if (n <= 0) break;
            total_flushed += n;
        }
        if (g_verbose) printf("Flushed %d bytes\n", total_flushed);
    }

    log_message("Step 2: Setting ULP active time to 255s (prevent sleep)...\n");
    send_cmd_wait_response(g_fd, 0x90, (uint8_t[]){0xFF}, 1, 1000);

    log_message("Step 3: Sending initialization commands...\n");
    send_cmd_wait_response(g_fd, 0xFE, NULL, 0, 1000);
    send_cmd_wait_response(g_fd, 0xD0, NULL, 0, 1000);
    send_cmd_wait_response(g_fd, 0xD2, NULL, 0, 1000);
    send_cmd_wait_response(g_fd, 0xD1, (uint8_t[]){0x01}, 1, 1000);

    log_message("Step 4: Reading all responses...\n");
    {
        struct timeval start, now;
        gettimeofday(&start, NULL);
        while (g_running) {
            fd_set rfds;
            struct timeval tv;
            FD_ZERO(&rfds);
            FD_SET(g_fd, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            int ret = select(g_fd + 1, &rfds, NULL, NULL, &tv);
            if (ret > 0 && FD_ISSET(g_fd, &rfds)) {
                receive_and_process(g_fd);
            }
            gettimeofday(&now, NULL);
            if ((now.tv_sec - start.tv_sec) >= 3) break;
        }
    }

    log_message("Step 9: Entering main loop - waiting for BSD reports...\n");

    int last_gpio_val = -1;
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
            fprintf(stderr, "select error: %s\n", strerror(errno));
            break;
        }

        if (ret > 0 && FD_ISSET(g_fd, &rfds)) {
            receive_and_process(g_fd);
        }

        if (gpio_line_fd >= 0) {
            loop_count++;
            if (loop_count >= 10) {
                loop_count = 0;
                int val = read_gpio_out(gpio_line_fd);
                if (val >= 0 && val != last_gpio_val) {
                    log_message("Radar OUT (PB2) changed: %d -> %d\n", last_gpio_val, val);
                    last_gpio_val = val;
                }
            }
        }
    }

    log_message("Radar BSD Detection Program stopping\n");

    if (gpio_line_fd >= 0) close(gpio_line_fd);
    if (g_gpio_fd >= 0) close(g_gpio_fd);
    if (g_log_file) fclose(g_log_file);
    close(g_fd);

    return 0;
}
