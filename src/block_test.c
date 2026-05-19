#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/time.h>

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

int main()
{
    int fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return 1; }

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
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);

    for (int test = 1; test <= 10; test++) {
        tcflush(fd, TCIOFLUSH);
        usleep(50000);

        uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
        write(fd, cmd, sizeof(cmd));
        tcdrain(fd);

        uint8_t rx[64] = {0};
        int total = 0;
        struct timeval start, now;
        gettimeofday(&start, NULL);

        while (total < 20) {
            int n = read(fd, rx + total, sizeof(rx) - total);
            if (n > 0) {
                total += n;
            } else if (n == 0) {
                break;
            } else {
                if (errno == EAGAIN) {
                    usleep(1000);
                    gettimeofday(&now, NULL);
                    int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                                  (now.tv_usec - start.tv_usec) / 1000;
                    if (total > 0 && elapsed >= 200) break;
                    if (elapsed >= 500) break;
                    continue;
                }
                break;
            }
            gettimeofday(&now, NULL);
            int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_usec - start.tv_usec) / 1000;
            if (total > 0 && elapsed >= 200) break;
            if (elapsed >= 500) break;
        }

        printf("Test %d: RX %d bytes: ", test, total);
        print_hex(rx, total > 16 ? 16 : total);
        if (total > 16) printf("...");
        if (total > 0 && rx[0] == 0x59) printf(" *** VALID! ***");
        printf("\n");
    }

    close(fd);
    return 0;
}