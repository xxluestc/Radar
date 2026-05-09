#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>

static int uart_open(const char *device, int baudrate)
{
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    speed_t speed = B921600;
    switch (baudrate) {
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default: speed = B921600; break;
    }

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

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    return fd;
}

int main(int argc, char *argv[])
{
    const char *device = "/dev/ttySTM3";
    int baudrate = 921600;

    if (argc > 1) device = argv[1];
    if (argc > 2) baudrate = atoi(argv[2]);

    printf("Opening %s at %d baud...\n", device, baudrate);

    int fd = uart_open(device, baudrate);
    if (fd < 0) {
        fprintf(stderr, "Failed to open UART\n");
        return 1;
    }

    printf("UART opened successfully (fd=%d)\n", fd);

    sleep(1);

    printf("Sending test data...\n");
    const char *test_data = "LOOPBACK_TEST_12345";
    ssize_t written = write(fd, test_data, strlen(test_data));
    printf("Written %zd bytes: \"%s\"\n", written, test_data);

    tcdrain(fd);

    usleep(200000);

    printf("Reading response...\n");
    char buf[256];
    memset(buf, 0, sizeof(buf));

    int total_read = 0;
    int attempts = 10;
    while (attempts-- > 0) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        int ret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ret > 0) {
            ssize_t n = read(fd, buf + total_read, sizeof(buf) - total_read - 1);
            if (n > 0) {
                total_read += n;
                printf("Read %zd bytes (total: %d)\n", n, total_read);
            } else if (n == 0) {
                printf("Read returned 0\n");
            } else {
                printf("Read error: %s\n", strerror(errno));
            }
        } else if (ret == 0) {
            printf("Timeout (attempts left: %d)\n", attempts);
        } else {
            printf("Select error: %s\n", strerror(errno));
        }

        if (total_read > 0 && attempts < 5)
            break;
    }

    if (total_read > 0) {
        buf[total_read] = '\0';
        printf("\n=== LOOPBACK SUCCESS ===\n");
        printf("Received %d bytes: \"%s\"\n", total_read, buf);
        printf("Hex: ");
        for (int i = 0; i < total_read; i++)
            printf("%02X ", (unsigned char)buf[i]);
        printf("\n");
    } else {
        printf("\n=== LOOPBACK FAILED ===\n");
        printf("No data received!\n");
    }

    close(fd);
    return total_read > 0 ? 0 : 1;
}
