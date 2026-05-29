/*
 * MS60-1211S80M-BSD (AT6010) 雷达测试程序 — 飞腾派 PE2204 版
 * 基于 radar_test.c (MP257版) 改写，仅修改串口设备路径
 * 雷达协议和配置命令与开发板无关，应用层代码完全一致
 *
 * 飞腾派串口映射:
 *   - J1 DEBUG_UART1 (Pin7=TXD, Pin9=RXD) → /dev/ttyAMA1 (0x2800D000) [console占用]
 *   - 40pin/J1 UART2 (Pin8=TXD, Pin10=RXD) → /dev/ttyAMA2 (0x2800E000)
 *
 * 编译 (交叉编译):
 *   export PATH=/home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin:$PATH
 *   aarch64-none-linux-gnu-gcc -Wall -O2 -o radar_phytium_test radar_phytium_test.c
 *
 * 运行:
 *   ./radar_phytium_test              默认 /dev/ttyAMA2 @ 921600
 *   ./radar_phytium_test /dev/ttyAMA2 115200
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/select.h>

static uint16_t calc_sum16(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

static void build_cmd(uint8_t *frame, int *len, int group, int cmd, const uint8_t *params, int param_len)
{
    int idx = 0;
    frame[idx++] = 0x58;
    frame[idx++] = (group << 5) | (cmd & 0x1F);
    frame[idx++] = (uint8_t)param_len;
    if (params && param_len > 0) {
        memcpy(&frame[idx], params, param_len);
        idx += param_len;
    }
    uint16_t csum = calc_sum16(frame, idx);
    frame[idx++] = (uint8_t)(csum & 0xFF);
    frame[idx++] = (uint8_t)((csum >> 8) & 0xFF);
    *len = idx;
}

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static int send_and_recv(int fd, int group, int cmd, const uint8_t *params, int param_len, const char *name)
{
    uint8_t frame[128];
    int flen;
    build_cmd(frame, &flen, group, cmd, params, param_len);

    printf("SEND [%s]: ", name);
    print_hex(frame, flen);
    printf("\n");

    tcflush(fd, TCIOFLUSH);
    write(fd, frame, flen);
    tcdrain(fd);

    usleep(300000);

    uint8_t buf[1024];
    int total = 0;
    struct timeval tv = {0, 500000};
    fd_set rfds;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    int ret = select(fd + 1, &rfds, NULL, NULL, &tv);

    if (ret > 0) {
        total = read(fd, buf, sizeof(buf));
    }

    if (total > 0) {
        printf("RECV [%s] (%d bytes): ", name, total);
        print_hex(buf, total);
        printf("\n");

        if (total >= 5 && buf[0] == 0x59) {
            uint16_t csum = calc_sum16(buf, total - 2);
            uint16_t expected = buf[total - 2] | (buf[total - 1] << 8);
            if (csum == expected) {
                int grp = buf[1] >> 5;
                int c = buf[1] & 0x1F;
                int plen = buf[2];
                printf("  -> G%d.0x%02X Len=%d CKSUM_OK\n", grp, c, plen);
                return 0;
            } else {
                printf("  -> CKSUM ERR (got %04X, expected %04X)\n", expected, csum);
            }
        }
        return 0;
    } else {
        printf("RECV [%s]: NONE\n", name);
    }
    return -1;
}

static int set_uart(int fd, int baudrate)
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        printf("tcgetattr failed: %s\n", strerror(errno));
        return -1;
    }

    cfmakeraw(&tty);

    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
            printf("Unsupported baudrate: %d\n", baudrate);
            return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 3;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("tcsetattr failed: %s\n", strerror(errno));
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return 0;
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/ttyAMA2";
    int baudrate = argc > 2 ? atoi(argv[2]) : 921600;

    printf("=== Radar Test — Phytium PE2204 ===\n");
    printf("Device: %s\n", device);
    printf("Baud:   %d\n\n", baudrate);

    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("FAIL: cannot open %s (%s)\n", device, strerror(errno));
        return 1;
    }

    if (set_uart(fd, baudrate) != 0) {
        close(fd);
        return 1;
    }
    printf("UART configured: %d/8N1\n\n", baudrate);

    printf("=== Step 1: Read version ===\n");
    send_and_recv(fd, 7, 0x1E, NULL, 0, "VERSION");

    printf("\n=== Step 2: Read status ===\n");
    send_and_recv(fd, 1, 0x01, NULL, 0, "STATUS");

    uint8_t param_1 = 0x01;
    printf("\n=== Step 3: Enable sensor ===\n");
    send_and_recv(fd, 6, 0x11, &param_1, 1, "ENABLE_SENSE");

    printf("\n=== Step 4: Enable BSD detection ===\n");
    send_and_recv(fd, 6, 0x10, &param_1, 1, "ENABLE_BSD");

    printf("\n=== Step 5: Enable auto report ===\n");
    send_and_recv(fd, 6, 0x12, &param_1, 1, "ENABLE_AUTO");

    printf("\n=== Step 6: Monitor BSD data (Ctrl+C to stop) ===\n");
    while (1) {
        struct timeval tv = {1, 0};
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0) {
            uint8_t buf[256];
            int n = read(fd, buf, sizeof(buf));
            if (n > 0) {
                printf("DATA (%d bytes): ", n);
                print_hex(buf, n);
                printf("\n");
            }
        } else {
            printf(".");
            fflush(stdout);
        }
    }

    close(fd);
    return 0;
}
