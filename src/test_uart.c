#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>

int main(int argc, char *argv[])
{
    int baudrate = 921600;
    const char *device = "/dev/ttySTM3";

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", device, strerror(errno));
        return 1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
    speed_t speed = B921600;
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
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);

    printf("Sending system reset command...\n");
    uint8_t reset_cmd[] = {0x58, 0x13, 0x01, 0x01, 0x6D, 0x00};
    write(fd, reset_cmd, sizeof(reset_cmd));
    tcdrain(fd);

    printf("Waiting for radar to reboot (5s)...\n");
    {
        struct timeval st, nw;
        gettimeofday(&st, NULL);
        while (1) {
            gettimeofday(&nw, NULL);
            if ((nw.tv_sec - st.tv_sec) >= 5) break;
            usleep(100000);
        }
    }

    printf("Sending get version command...\n");
    uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
    write(fd, cmd, sizeof(cmd));
    tcdrain(fd);

    printf("Waiting for response...\n");
    usleep(100000);

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
        tv.tv_usec = 50000;

        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(fd, &rfds)) {
            int n = read(fd, buf + total, sizeof(buf) - total);
            if (n > 0) {
                total += n;
            }
        }

        gettimeofday(&now, NULL);
        if ((now.tv_sec - start.tv_sec) >= 2) break;
    }

    printf("Received %d bytes:\n", total);
    for (int i = 0; i < total; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");

    close(fd);
    return 0;
}
