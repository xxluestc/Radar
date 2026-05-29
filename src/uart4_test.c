#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>

#define UART4_BASE  0x40100000
#define GPIOB_BASE  0x44250000

int main() {
    printf("=== UART4 Loopback Test ===\n\n");

    int tty_fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY);
    if (tty_fd < 0) { perror("open"); return 1; }
    printf("Opened /dev/ttySTM3\n");

    struct termios tty;
    tcgetattr(tty_fd, &tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    tty.c_cflag = CS8 | CREAD | CLOCAL;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 20;
    tcsetattr(tty_fd, TCSANOW, &tty);
    tcflush(tty_fd, TCIOFLUSH);
    printf("Configured 921600 8N1\n");

    int mem_fd = open("/dev/mem", O_RDWR);
    volatile uint32_t *uart = mmap(NULL, 0x400, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, UART4_BASE);
    volatile uint32_t *gpio = mmap(NULL, 0x400, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIOB_BASE);

    printf("\n--- UART4 Registers ---\n");
    printf("CR1=0x%08X  CR3=0x%08X  BRR=%u\n", uart[0x00/4], uart[0x08/4], uart[0x0C/4]);
    uint32_t isr = uart[0x1C/4];
    printf("ISR=0x%08X (TXE=%d TC=%d RXNE=%d FE=%d)\n", isr, (isr>>7)&1, (isr>>6)&1, (isr>>5)&1, (isr>>1)&1);

    printf("\n--- GPIOB Status ---\n");
    uint32_t idr = gpio[0x10/4];
    printf("IDR=0x%08X  PB6(RX)=%d  PB7(TX)=%d\n", idr, (idr>>6)&1, (idr>>7)&1);

    printf("\n--- Send Test ---\n");
    char tx[] = "HELLO_UART4_TEST_1234567890";
    int n = write(tty_fd, tx, strlen(tx));
    printf("write() returned %d, wrote: %s\n", n, tx);

    usleep(100000);

    isr = uart[0x1C/4];
    idr = gpio[0x10/4];
    printf("After TX: ISR=0x%08X (TXE=%d TC=%d RXNE=%d)\n", isr, (isr>>7)&1, (isr>>6)&1, (isr>>5)&1);

    printf("\n--- Receive Test ---\n");
    char rx[256];
    memset(rx, 0, sizeof(rx));
    n = read(tty_fd, rx, sizeof(rx)-1);
    printf("read() returned %d bytes: ", n);
    if (n > 0) {
        for (int i = 0; i < n; i++) printf("%02X ", (unsigned char)rx[i]);
        printf("\nASCII: %s\n", rx);
    } else {
        printf("NO DATA\n");
    }

    isr = uart[0x1C/4];
    printf("After RX: ISR=0x%08X (RXNE=%d FE=%d)\n", isr, (isr>>5)&1, (isr>>1)&1);

    munmap((void*)uart, 0x400);
    munmap((void*)gpio, 0x400);
    close(mem_fd);
    close(tty_fd);

    printf("\n=== Test Complete ===\n");
    return 0;
}