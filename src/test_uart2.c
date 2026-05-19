#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>

int main(int argc, char *argv[])
{
    const char *device = "/dev/ttySTM3";

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", device, strerror(errno));
        return 1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
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
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);

    printf("Test 1: Send command then receive (original method)\n");
    uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
    write(fd, cmd, sizeof(cmd));
    tcdrain(fd);

    usleep(50000);

    uint8_t buf[256];
    int total = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);
    while (1) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, sizeof(buf) - total);
            if (n > 0) total += n;
        }
        gettimeofday(&now, NULL);
        if ((now.tv_sec - start.tv_sec) >= 2) break;
    }

    printf("Received %d bytes: ", total);
    for (int i = 0; i < total; i++) printf("%02X ", buf[i]);
    printf("\n");

    sleep(1);

    printf("\nTest 2: Send command with tcdrain, then small delay, then receive\n");
    tcflush(fd, TCIOFLUSH);
    write(fd, cmd, sizeof(cmd));
    tcdrain(fd);
    usleep(200000);

    total = 0;
    gettimeofday(&start, NULL);
    while (1) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, sizeof(buf) - total);
            if (n > 0) total += n;
        }
        gettimeofday(&now, NULL);
        if ((now.tv_sec - start.tv_sec) >= 2) break;
    }

    printf("Received %d bytes: ", total);
    for (int i = 0; i < total; i++) printf("%02X ", buf[i]);
    printf("\n");

    printf("\nTest 3: Send command, flush RX, wait 500ms, then receive\n");
    tcflush(fd, TCIOFLUSH);
    write(fd, cmd, sizeof(cmd));
    tcdrain(fd);
    usleep(500000);

    total = 0;
    gettimeofday(&start, NULL);
    while (1) {
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, sizeof(buf) - total);
            if (n > 0) total += n;
        }
        gettimeofday(&now, NULL);
        if ((now.tv_sec - start.tv_sec) >= 2) break;
    }

    printf("Received %d bytes: ", total);
    for (int i = 0; i < total; i++) printf("%02X ", buf[i]);
    printf("\n");
}
