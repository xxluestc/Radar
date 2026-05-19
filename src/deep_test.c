#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/select.h>

static int set_uart(int fd, int baudrate)
{
    struct termios tty;
    tcgetattr(fd, &tty);
    speed_t speed = B921600;
    switch (baudrate) {
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

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static uint16_t calc_checksum(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

int main()
{
    int fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }
    set_uart(fd, 921600);

    uint8_t tmp[256];
    while (read(fd, tmp, sizeof(tmp)) > 0) {}
    usleep(100000);

    uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
    printf("TX: "); print_hex(cmd, sizeof(cmd)); printf("\n");
    write(fd, cmd, sizeof(cmd));
    tcdrain(fd);

    uint8_t rx[1024];
    int total = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (total < (int)sizeof(rx) - 1) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 5000;
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, rx + total, sizeof(rx) - total);
            if (n > 0) total += n;
        }
        gettimeofday(&now, NULL);
        int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                      (now.tv_usec - start.tv_usec) / 1000;
        if (total > 0 && elapsed >= 500) break;
        if (elapsed >= 1000) break;
    }

    printf("RX (%d bytes): ", total);
    print_hex(rx, total);
    printf("\n");

    if (total > 0) {
        printf("First byte: 0x%02X (binary: ", rx[0]);
        for (int i = 7; i >= 0; i--) printf("%d", (rx[0] >> i) & 1);
        printf(")\n");

        if (rx[0] == 0x59) {
            printf("*** VALID RESPONSE HEADER! ***\n");
            if (total >= 5) {
                uint8_t plen = rx[2];
                int expected = 3 + plen + 2;
                printf("CMD=0x%02X, PLEN=%d, expected total=%d, actual=%d\n",
                       rx[1], plen, expected, total);
                if (total >= expected) {
                    uint16_t recv_ck = rx[3 + plen] | (rx[3 + plen + 1] << 8);
                    uint16_t calc_ck = calc_checksum(rx, 3 + plen);
                    printf("Checksum: recv=0x%04X, calc=0x%04X, %s\n",
                           recv_ck, calc_ck, recv_ck == calc_ck ? "MATCH!" : "MISMATCH");
                }
            }
        } else if (rx[0] == 0x58) {
            printf("Got COMMAND header (0x58) - radar might be echoing\n");
        } else if (rx[0] == 0x5A) {
            printf("Got REPORT header (0x5A) - BSD report!\n");
        }
    }

    close(fd);
    return 0;
}