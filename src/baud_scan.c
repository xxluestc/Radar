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

static int set_uart_raw(int fd, int baudrate)
{
    struct termios tty;
    tcgetattr(fd, &tty);
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
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

int main()
{
    int baudrates[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    int n_bauds = sizeof(baudrates) / sizeof(baudrates[0]);

    for (int i = 0; i < n_bauds; i++) {
        int fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) { perror("open"); return 1; }

        set_uart_raw(fd, baudrates[i]);

        uint8_t tmp[256];
        while (read(fd, tmp, sizeof(tmp)) > 0) {}
        usleep(50000);

        uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
        write(fd, cmd, sizeof(cmd));
        tcdrain(fd);

        uint8_t rx[256];
        int total = 0;
        struct timeval start, now;
        gettimeofday(&start, NULL);

        while (total < (int)sizeof(rx)) {
            fd_set rfds;
            struct timeval tv;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 10000;
            int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
            if (ret > 0 && FD_ISSET(fd, &rfds)) {
                int n = read(fd, rx + total, sizeof(rx) - total);
                if (n > 0) total += n;
            }
            gettimeofday(&now, NULL);
            int elapsed = (now.tv_sec - start.tv_sec) * 1000 +
                          (now.tv_usec - start.tv_usec) / 1000;
            if (elapsed >= 300) break;
        }

        printf("Baud %d: RX %d bytes: ", baudrates[i], total);
        print_hex(rx, total);
        printf("\n");

        if (total >= 5 && rx[0] == 0x59) {
            printf("  *** VALID RESPONSE at %d baud! ***\n", baudrates[i]);
        }

        close(fd);
        usleep(100000);
    }

    return 0;
}