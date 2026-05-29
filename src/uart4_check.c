#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/mman.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>

#define PAGE_SIZE 4096

volatile uint32_t *gpiob_base;
volatile uint32_t *uart4_base;
int tty_fd = -1;

void mmap_peri(uint32_t phys, volatile uint32_t **vaddr)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); exit(1); }
    void *p = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys & ~(PAGE_SIZE - 1));
    if (p == MAP_FAILED) { perror("mmap"); exit(1); }
    *vaddr = (volatile uint32_t *)(p + (phys & (PAGE_SIZE - 1)));
    close(fd);
}

static uint16_t calc_sum16(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (uint16_t)(sum & 0xFFFF);
}

static void print_hex(const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++) printf("%02X ", d[i]);
}

static void send_cmd(int fd, int grp, int cmd, uint8_t param)
{
    uint8_t frame[6];
    int idx = 0;
    frame[idx++] = 0x58;
    frame[idx++] = (grp << 5) | (cmd & 0x1F);
    frame[idx++] = 1;
    frame[idx++] = param;
    uint16_t csum = calc_sum16(frame, idx);
    frame[idx++] = (uint8_t)(csum & 0xFF);
    frame[idx++] = (uint8_t)((csum >> 8) & 0xFF);

    printf("TX: "); print_hex(frame, idx); printf("\n");
    tcflush(fd, TCIOFLUSH);
    write(fd, frame, idx);
    tcdrain(fd);
}

int main()
{
    printf("=== UART4 & GPIOB Register Check ===\n\n");

    signal(SIGALRM, SIG_DFL);
    alarm(5);

    /* mmap GPIOB and UART4 */
    mmap_peri(0x42000000, &gpiob_base);
    mmap_peri(0x40100000, &uart4_base);

    /* GPIOB registers at offsets */
#define MODER   (0x00/4)
#define OTYPER  (0x04/4)
#define OSPEEDR (0x08/4)
#define PUPDR   (0x0c/4)
#define IDR     (0x10/4)
#define AFRL    (0x20/4)

    uint32_t moder   = gpiob_base[MODER];
    uint32_t otyper  = gpiob_base[OTYPER];
    uint32_t ospeedr = gpiob_base[OSPEEDR];
    uint32_t pupdr   = gpiob_base[PUPDR];
    uint32_t idr     = gpiob_base[IDR];
    uint32_t afrl    = gpiob_base[AFRL];

    printf("[GPIOB] MODER   = 0x%08X\n", moder);
    printf("[GPIOB] OTYPER  = 0x%08X\n", otyper);
    printf("[GPIOB] OSPEEDR = 0x%08X\n", ospeedr);
    printf("[GPIOB] PUPDR   = 0x%08X\n", pupdr);
    printf("[GPIOB] IDR     = 0x%08X\n", idr);
    printf("[GPIOB] AFRL    = 0x%08X\n", afrl);

    int pb6_moder = (moder >> 12) & 3;
    int pb7_moder = (moder >> 14) & 3;
    int pb6_af    = (afrl >> 24) & 0xF;
    int pb7_af    = (afrl >> 28) & 0xF;
    int pb6_pupd  = (pupdr >> 12) & 3;
    int pb7_pupd  = (pupdr >> 14) & 3;
    int pb7_idr   = (idr >> 7) & 1;
    int pb6_ot    = (otyper >> 6) & 1;
    int pb7_ot    = (otyper >> 7) & 1;

    printf("\n=== Pin Analysis ===\n");
    printf("PB6 MODER=%d %s AF=%d %s PUPDR=%d %s OT=%d %s\n",
           pb6_moder, pb6_moder==2?"AF":"??",
           pb6_af, pb6_af==4?"AF3":"??",
           pb6_pupd, pb6_pupd==1?"PULL-UP":pb6_pupd==0?"NONE":"PULL-DOWN",
           pb6_ot, pb6_ot==0?"PP":"OD");
    printf("PB7 MODER=%d %s AF=%d %s PUPDR=%d %s OT=%d %s\n",
           pb7_moder, pb7_moder==2?"AF":"??",
           pb7_af, pb7_af==4?"AF3":"??",
           pb7_pupd, pb7_pupd==0?"NONE":pb7_pupd==1?"PULL-UP":"PULL-DOWN",
           pb7_ot, pb7_ot==0?"PP":"OD");
    printf("PB7 IDR=%d (radar TX level, should be 1=HIGH if powered)\n", pb7_idr);

    /* Open tty */
    printf("\n=== Opening /dev/ttySTM3 ===\n");
    tty_fd = open("/dev/ttySTM3", O_RDWR | O_NOCTTY);
    if (tty_fd < 0) {
        printf("FAIL: open errno=%d\n", errno);
        return 1;
    }
    printf("tty opened fd=%d\n", tty_fd);

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(tty_fd, &tty);
    cfmakeraw(&tty);
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(tty_fd, TCSANOW, &tty);
    printf("tty configured 921600/8N1\n");

    usleep(100000);

    /* UART4 registers */
#define UART_CR1   (0x00/4)
#define UART_CR2   (0x04/4)
#define UART_CR3   (0x08/4)
#define UART_BRR   (0x0c/4)
#define UART_GTPR  (0x10/4)
#define UART_RTOR  (0x14/4)
#define UART_RQR   (0x18/4)
#define UART_ISR   (0x1c/4)
#define UART_ICR   (0x20/4)
#define UART_RDR   (0x24/4)
#define UART_TDR   (0x28/4)
#define UART_PRESC (0x2c/4)

    uint32_t cr1   = uart4_base[UART_CR1];
    uint32_t cr2   = uart4_base[UART_CR2];
    uint32_t cr3   = uart4_base[UART_CR3];
    uint32_t brr   = uart4_base[UART_BRR];
    uint32_t presc = uart4_base[UART_PRESC];
    uint32_t isr   = uart4_base[UART_ISR];

    printf("\n=== UART4 Registers ===\n");
    printf("CR1   = 0x%08X (UE=%d RE=%d TE=%d RXNEIE=%d RTOIE=%d FIFOEN=%d OVER8=%d)\n",
           cr1, cr1&1, (cr1>>2)&1, (cr1>>3)&1, (cr1>>5)&1, (cr1>>26)&1, (cr1>>29)&1, (cr1>>15)&1);
    printf("CR2   = 0x%08X (SWAP=%d)\n", cr2, (cr2>>23)&1);
    printf("CR3   = 0x%08X (DMAR=%d DMAT=%d)\n", cr3, (cr3>>6)&1, (cr3>>7)&1);
    printf("BRR   = 0x%08X (M=%u F=%u USARTDIV=%u)\n", brr,
           (unsigned)((brr >> 4) & 0xFFF), (unsigned)(brr & 0xF),
           (unsigned)(((brr>>4)&0xFFF)*16 + (brr&0xF)));
    printf("PRESC = 0x%08X\n", presc);
    printf("ISR   = 0x%08X (TXE=%d TC=%d RXNE=%d ORE=%d FE=%d PE=%d)\n",
           isr, (isr>>7)&1, (isr>>6)&1, (isr>>5)&1, (isr>>3)&1, (isr>>1)&1, isr&1);

    /* Check each config item */
    printf("\n=== Configuration Check Results ===\n");
    int ok = 1;

    if (pb6_moder == 2) printf("[OK] PB6 MODER=AF\n");
    else { printf("[FAIL] PB6 MODER=%d (need 2)\n", pb6_moder); ok = 0; }

    if (pb7_moder == 2) printf("[OK] PB7 MODER=AF\n");
    else { printf("[FAIL] PB7 MODER=%d (need 2)\n", pb7_moder); ok = 0; }

    if (pb6_af == 4) printf("[OK] PB6 AF=AF3 (val=4)\n");
    else { printf("[FAIL] PB6 AF=%d (need 4)\n", pb6_af); ok = 0; }

    if (pb7_af == 4) printf("[OK] PB7 AF=AF3 (val=4)\n");
    else { printf("[FAIL] PB7 AF=%d (need 4)\n", pb7_af); ok = 0; }

    if ((cr2 >> 23) & 1) printf("[OK] UART4 SWAP=1\n");
    else { printf("[FAIL] SWAP=0\n"); ok = 0; }

    if (cr1 & 1) printf("[OK] UE=1 (UART enabled)\n");
    else { printf("[FAIL] UE=0\n"); ok = 0; }

    if ((cr1>>2)&1) printf("[OK] RE=1 (RX enabled)\n");
    else { printf("[FAIL] RE=0\n"); ok = 0; }

    if ((cr1>>3)&1) printf("[OK] TE=1 (TX enabled)\n");
    else { printf("[FAIL] TE=0\n"); ok = 0; }

    uint32_t usartdiv = ((brr>>4)&0xFFF)*16 + (brr&0xF);
    uint32_t calc_baud = 64000000 / usartdiv;
    printf("[INFO] BRR USARTDIV=%u Baud=~%u (target 921600)\n", (unsigned)usartdiv, (unsigned)calc_baud);

    if (pb7_idr == 1) printf("[OK] Radar TX pin is HIGH (radar powered, idle)\n");
    else printf("[WARN] Radar TX pin is LOW (%d) - radar may not be powered\n", pb7_idr);

    /* Try send command and check RX */
    printf("\n=== Send VERSION command ===\n");
    send_cmd(tty_fd, 7, 0x1E, 0);

    /* Poll for RX data */
    int retry = 0;
    for (retry = 0; retry < 30; retry++) {
        usleep(100000);
        uint32_t isr_now = uart4_base[UART_ISR];
        if ((isr_now >> 5) & 1) {
            uint32_t rdr = uart4_base[UART_RDR];
            printf("RXNE=1! RDR=0x%02X at retry=%d\n", rdr, retry);
            break;
        }
        if (retry % 5 == 4) printf("  poll %d: RXNE=%d ISR=0x%08X\n",
                                    retry, (isr_now>>5)&1, isr_now);
    }

    if (retry >= 30) {
        printf("No RX data after 3 seconds.\n");
    }

    printf("\n=== Summary: %s ===\n", ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");

    if (tty_fd >= 0) close(tty_fd);
    return ok ? 0 : 1;
}