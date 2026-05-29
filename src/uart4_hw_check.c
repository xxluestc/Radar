#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>

#define UART4_BASE  0x40100000
#define GPIOB_BASE  0x44250000
#define GPIO_MODER  (GPIOB_BASE + 0x00)
#define GPIO_OTYPER (GPIOB_BASE + 0x04)
#define GPIO_OSPEEDR (GPIOB_BASE + 0x08)
#define GPIO_PUPDR  (GPIOB_BASE + 0x0C)
#define GPIO_IDR    (GPIOB_BASE + 0x10)
#define GPIO_ODR    (GPIOB_BASE + 0x14)
#define GPIO_AFRL   (GPIOB_BASE + 0x20)
#define GPIO_AFRH   (GPIOB_BASE + 0x24)

#define UART_CR1    (UART4_BASE + 0x00)
#define UART_CR2    (UART4_BASE + 0x04)
#define UART_CR3    (UART4_BASE + 0x08)
#define UART_BRR    (UART4_BASE + 0x0C)
#define UART_GTPR   (UART4_BASE + 0x10)
#define UART_RTOR   (UART4_BASE + 0x14)
#define UART_RQR    (UART4_BASE + 0x18)
#define UART_ISR    (UART4_BASE + 0x1C)
#define UART_ICR    (UART4_BASE + 0x20)
#define UART_RDR    (UART4_BASE + 0x24)
#define UART_TDR    (UART4_BASE + 0x28)

#define REG32(addr)  (*((volatile uint32_t*)(addr)))
#define WRITE32(addr, val)  (*((volatile uint32_t*)(addr)) = (val))

int main() {
    int mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) { perror("open /dev/mem"); return 1; }

    void *uart_map = mmap(NULL, 0x400, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, UART4_BASE);
    void *gpio_map = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIOB_BASE);

    if (uart_map == MAP_FAILED || gpio_map == MAP_FAILED) {
        perror("mmap"); return 1;
    }

    volatile uint32_t *uart = (volatile uint32_t*)uart_map;
    volatile uint32_t *gpio = (volatile uint32_t*)gpio_map;

    // First, open tty to enable UART4 clock
    printf("Opening /dev/ttySTM3 to enable clock...\n");
    int tty_fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY);
    if (tty_fd < 0) { perror("open tty"); return 1; }
    usleep(100000);

    // Print GPIO state
    uint32_t moder    = gpio[0x00/4];
    uint32_t otyper   = gpio[0x04/4];
    uint32_t ospeedr  = gpio[0x08/4];
    uint32_t pupdr    = gpio[0x0C/4];
    uint32_t idr      = gpio[0x10/4];
    uint32_t odr      = gpio[0x14/4];
    uint32_t afrl     = gpio[0x20/4];
    uint32_t afrh     = gpio[0x24/4];

    printf("\n=== GPIOB Status ===\n");
    printf("MODER=0x%08X OTYPER=0x%08X OSPEEDR=0x%08X PUPDR=0x%08X\n", moder, otyper, ospeedr, pupdr);
    printf("IDR=0x%08X ODR=0x%08X AFRL=0x%08X AFRH=0x%08X\n", idr, odr, afrl, afrh);

    int pb6_mode = (moder >> 12) & 3;
    int pb7_mode = (moder >> 14) & 3;
    int pb6_af = (afrl >> 24) & 0xF;
    int pb7_af = (afrl >> 28) & 0xF;
    const char *mode_names[] = {"INPUT", "OUTPUT", "AF", "ANALOG"};
    printf("PB6: MODE=%s AF=%d IN=%d\n", mode_names[pb6_mode], pb6_af, (idr>>6)&1);
    printf("PB7: MODE=%s AF=%d IN=%d ODR=%d\n", mode_names[pb7_mode], pb7_af, (idr>>7)&1, (odr>>7)&1);

    // Print UART4 status
    printf("\n=== UART4 Status ===\n");
    uint32_t cr1 = uart[0x00/4];
    uint32_t cr2 = uart[0x04/4];
    uint32_t cr3 = uart[0x08/4];
    uint32_t brr = uart[0x0C/4];
    uint32_t isr = uart[0x1C/4];
    uint32_t rdr = uart[0x24/4];

    printf("CR1=0x%08X CR2=0x%08X CR3=0x%08X BRR=%u\n", cr1, cr2, cr3, brr);
    printf("ISR=0x%08X RDR=0x%02X\n", isr, rdr & 0xFF);
    printf("UE=%d TE=%d RE=%d FIFOEN=%d\n", cr1&1, (cr1>>3)&1, (cr1>>2)&1, (cr1>>29)&1);
    printf("TXE=%d TC=%d RXNE=%d FE=%d\n", (isr>>7)&1, (isr>>6)&1, (isr>>5)&1, (isr>>1)&1);

    // Try toggling PB7 as GPIO output and read PB6 and UART RDR
    printf("\n=== Manual GPIO Toggle Test ===\n");

    // Save original MODER
    uint32_t moder_orig = moder;

    // Set PB7 as GPIO output
    gpio[0x00/4] = (moder & ~(3 << 14)) | (1 << 14);
    usleep(1000);
    printf("PB7 set to OUTPUT. MODER=0x%08X\n", gpio[0x00/4]);

    // Toggle HIGH
    gpio[0x14/4] = odr | (1 << 7);
    usleep(1000);
    idr = gpio[0x10/4];
    printf("PB7=HIGH: PB6=%d PB7=%d\n", (idr>>6)&1, (idr>>7)&1);

    // Toggle LOW
    gpio[0x14/4] = odr & ~(1 << 7);
    usleep(1000);
    idr = gpio[0x10/4];
    printf("PB7=LOW:  PB6=%d PB7=%d\n", (idr>>6)&1, (idr>>7)&1);

    // Toggle HIGH again
    gpio[0x14/4] = odr | (1 << 7);
    usleep(1000);
    idr = gpio[0x10/4];
    printf("PB7=HIGH: PB6=%d PB7=%d\n", (idr>>6)&1, (idr>>7)&1);

    // Restore PB7 to AF mode
    gpio[0x00/4] = moder_orig;
    usleep(1000);
    printf("PB7 restored to AF. MODER=0x%08X\n", gpio[0x00/4]);

    // Now check UART4 TX with manual register write
    printf("\n=== Manual UART TX Test ===\n");

    // Disable UART, re-configure
    uart[0x00/4] = 0;
    usleep(10000);
    uart[0x20/4] = 0xFFFFFFFF;
    uart[0x0C/4] = 555;  // 115200 baud
    uart[0x00/4] = 1 | 8 | 4;  // UE|TE|RE, no FIFO
    usleep(10000);

    cr1 = uart[0x00/4];
    isr = uart[0x1C/4];
    printf("After config: CR1=0x%08X ISR=0x%08X\n", cr1, isr);
    printf("TXE=%d TC=%d\n", (isr>>7)&1, (isr>>6)&1);

    // Check TX pin level
    idr = gpio[0x10/4];
    printf("PB7(TX)=%d (should be HIGH if TE=1 and idle)\n", (idr>>7)&1);

    // Send a byte
    if (isr & (1 << 7)) {
        printf("Writing 0xAA to TDR...\n");
        uart[0x28/4] = 0xAA;

        // Check TX pin immediately
        for (int i = 0; i < 20; i++) {
            usleep(10);
            idr = gpio[0x10/4];
            isr = uart[0x1C/4];
            int pb7_val = (idr >> 7) & 1;
            if (pb7_val == 1) {
                printf("  PB7 went HIGH at t=%dus!\n", (i+1)*10);
                break;
            }
            if (i == 19) {
                printf("  PB7 NEVER went HIGH!\n");
            }
        }

        usleep(50000);
        isr = uart[0x1C/4];
        rdr = uart[0x24/4];
        idr = gpio[0x10/4];
        printf("After TX: ISR=0x%08X RDR=0x%02X PB7=%d\n", isr, rdr&0xFF, (idr>>7)&1);
    }

    // Restore
    uart[0x00/4] = 0;

    munmap(uart_map, 0x400);
    munmap(gpio_map, 0x1000);
    close(mem_fd);
    close(tty_fd);

    return 0;
}