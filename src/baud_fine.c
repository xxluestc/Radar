#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <termios.h>

#define NCCS 19
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[NCCS];
    speed_t c_ispeed;
    speed_t c_ospeed;
};

#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#define BOTHER 0x1000

static void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) printf("%02X ", data[i]);
}

static int set_custom_baud(int fd, int baudrate)
{
    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) < 0) { perror("TCGETS2"); return -1; }
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= CREAD | CLOCAL;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    if (ioctl(fd, TCSETS2, &tio) < 0) { perror("TCSETS2"); return -1; }
    return 0;
}

int main()
{
    int fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }

    int bauds[] = {
        900000, 905000, 910000, 912000, 914000, 915000, 916000, 917000,
        918000, 919000, 920000, 921000, 921600, 922000, 923000, 924000,
        925000, 926000, 927000, 928000, 929000, 930000, 935000, 940000,
        950000, 960000, 970000, 980000, 990000, 1000000,
        0
    };

    for (int i = 0; bauds[i]; i++) {
        set_custom_baud(fd, bauds[i]);

        uint8_t tmp[256];
        while (read(fd, tmp, sizeof(tmp)) > 0) {}
        usleep(50000);

        uint8_t cmd[] = {0x58, 0xFE, 0x00, 0x56, 0x01};
        write(fd, cmd, sizeof(cmd));
        fsync(fd);

        uint8_t rx[256];
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
            if (total > 0 && elapsed >= 300) break;
            if (elapsed >= 500) break;
        }

        if (total > 0) {
            printf("Baud %d: RX %d bytes: ", bauds[i], total);
            print_hex(rx, total > 20 ? 20 : total);
            if (total > 20) printf("...");
            if (rx[0] == 0x59) printf(" *** VALID HEADER! ***");
            printf("\n");
        } else {
            printf("Baud %d: no response\n", bauds[i]);
        }
    }

    close(fd);
    return 0;
}