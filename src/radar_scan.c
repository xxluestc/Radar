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

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B921600;
    }
}

static uint16_t calc_sum16(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

static void build_cmd(uint8_t *frame, int *len, int group, int cmd,
                       const uint8_t *params, int param_len)
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

static int open_uart(const char *device, int baud)
{
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    cfmakeraw(&tty);
    speed_t spd = baud_to_speed(baud);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 3;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/ttySTM3";
    int bauds[] = {921600, 115200, 460800, 230400, 57600, 38400, 19200, 9600, 256000};
    int nb = sizeof(bauds) / sizeof(bauds[0]);

    printf("=== Radar Baud Rate Scan ===\nDevice: %s\n\n", device);

    for (int i = 0; i < nb; i++) {
        int baud = bauds[i];
        printf("--- Baud %d ---\n", baud);

        int fd = open_uart(device, baud);
        if (fd < 0) { printf("  FAIL open\n"); continue; }

        usleep(100000);

        uint8_t buf[256];
        int n;

        n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            printf("  GARBAGE (%d bytes): ", n);
            print_hex(buf, n > 32 ? 32 : n);
            printf("\n");
        }

        uint8_t frame[128];
        int flen;
        build_cmd(frame, &flen, 7, 0x1E, NULL, 0);
        tcflush(fd, TCIOFLUSH);
        write(fd, frame, flen);
        tcdrain(fd);

        usleep(300000);

        struct timeval tv = {0, 300000};
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        if (select(fd + 1, &rfds, NULL, NULL, &tv) > 0) {
            n = read(fd, buf, sizeof(buf));
            if (n > 0) {
                printf("  RECV (%d bytes): ", n);
                print_hex(buf, n > 32 ? 32 : n);
                if (n >= 5 && buf[0] == 0x59) {
                    uint16_t csum = calc_sum16(buf, n - 2);
                    uint16_t exp = buf[n-2] | (buf[n-1] << 8);
                    int grp = buf[1] >> 5;
                    int c = buf[1] & 0x1F;
                    printf("\n  *** HIT! *** G%d.0x%02X Len=%d CKSUM=%s at BAUD=%d",
                           grp, c, buf[2], csum == exp ? "OK" : "ERR", baud);
                }
                printf("\n");
            }
        } else {
            printf("  (no reply)\n");
        }

        close(fd);
    }

    printf("\n=== Scan Complete ===\n");
    return 0;
}