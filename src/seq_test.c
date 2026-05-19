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

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static int read_response(int fd, uint8_t *rx, int max_len, int timeout_ms)
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
            int n = read(fd, rx + total, max_len - total);
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

int main()
{
    int fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);

    for (int round = 1; round <= 5; round++) {
        printf("\n=== Round %d ===\n", round);

        for (int cmd_idx = 0; cmd_idx < 2; cmd_idx++) {
            uint8_t tmp[256];
            while (read(fd, tmp, sizeof(tmp)) > 0) {}
            usleep(50000);

            uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
            write(fd, cmd, sizeof(cmd));
            tcdrain(fd);

            uint8_t rx[256];
            int total = read_response(fd, rx, sizeof(rx), 500);

            printf("  Cmd%d: RX %d bytes: ", cmd_idx + 1, total);
            print_hex(rx, total > 16 ? 16 : total);
            if (total > 16) printf("...");
            if (total > 0 && rx[0] == 0x59) printf(" *** VALID! ***");
            printf("\n");
        }
    }

    close(fd);
    return 0;
}