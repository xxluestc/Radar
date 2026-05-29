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
    for (int i = 0; i < len && i < 64; i++) printf("%02X ", data[i]);
    if (len > 64) printf("...");
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
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/ttySTM3";
    int baud = argc > 2 ? atoi(argv[2]) : 921600;

    printf("=== Radar Passive Listen ===\n");
    printf("Device: %s, Baud: %d\n\n", device, baud);

    int fd = open_uart(device, baud);
    if (fd < 0) {
        printf("FAIL open\n");
        return 1;
    }
    printf("Listening for 3 seconds first...\n");
    usleep(100000);

    uint8_t buf[4096];
    int prefetch = read(fd, buf, sizeof(buf));
    if (prefetch > 0) {
        printf("Pre-existing data (%d bytes): ", prefetch);
        print_hex(buf, prefetch);
        printf("\n");
    }

    usleep(2000000);
    int n = read(fd, buf, sizeof(buf));
    if (n > 0) {
        printf("Passive data (%d bytes): ", n);
        print_hex(buf, n);
        printf("\n");
    } else {
        printf("No passive data. Sending commands...\n");
    }

    printf("\n=== Send VERSION command ===\n");
    uint8_t frame[128];
    int flen;
    build_cmd(frame, &flen, 7, 0x1E, NULL, 0);
    printf("TX: "); print_hex(frame, flen); printf("\n");
    tcflush(fd, TCIOFLUSH);
    write(fd, frame, flen);
    tcdrain(fd);

    usleep(500000);
    n = read(fd, buf, sizeof(buf));
    printf("RX (%d bytes): ", n);
    if (n > 0) { print_hex(buf, n); printf("\n"); }
    else { printf("NONE\n"); }

    close(fd);
    return 0;
}