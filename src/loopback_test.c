#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

int main(int argc, char *argv[]) {
    const char *dev = "/dev/ttySTM3";
    int baud = 921600;
    int fd;
    struct termios tty;
    char tx_data[] = "HELLO_LOOPBACK_TEST_12345";
    char rx_buf[256];
    int i;

    if (argc > 1) dev = argv[1];

    printf("Opening %s at %d baud...\n", dev, baud);
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tty);

    printf("Port configured. Sending %ld bytes: '%s'\n", strlen(tx_data), tx_data);
    int written = write(fd, tx_data, strlen(tx_data));
    printf("Wrote %d bytes.\n", written);
    tcdrain(fd);

    usleep(100000);

    memset(rx_buf, 0, sizeof(rx_buf));
    int total = 0;
    for (i = 0; i < 10; i++) {
        int n = read(fd, rx_buf + total, sizeof(rx_buf) - 1 - total);
        if (n > 0) {
            total += n;
            printf("Read %d bytes (total=%d): ", n, total);
            for (int j = total - n; j < total; j++)
                printf("%02X ", (unsigned char)rx_buf[j]);
            printf("\n");
        } else if (n == 0) {
            break;
        }
        usleep(50000);
    }

    if (total > 0) {
        rx_buf[total] = '\0';
        printf("Received: '%s'\n", rx_buf);
        if (strcmp(tx_data, rx_buf) == 0) {
            printf(">>> LOOPBACK SUCCESS! Data matches. <<<\n");
        } else {
            printf(">>> DATA MISMATCH! <<<\n");
        }
    } else {
        printf(">>> NO DATA RECEIVED (loopback FAILED) <<<\n");
    }

    close(fd);
    return (total > 0 && strcmp(tx_data, rx_buf) == 0) ? 0 : 1;
}