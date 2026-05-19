CROSS_COMPILE = aarch64-none-linux-gnu-
CC = $(CROSS_COMPILE)gcc
STRIP = $(CROSS_COMPILE)strip

TOOLCHAIN_PATH = /home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin
export PATH := $(TOOLCHAIN_PATH):$(PATH)

CFLAGS = -Wall -O2 -static

SRC_DIR = src
TARGETS = radar_bsd radar_bsd_v2 radar_bsd_v3 radar_bsd_v4 baud_scan deep_test baud_fine block_test seq_test time_test

.PHONY: all clean deploy

all: $(TARGETS)

radar_bsd: $(SRC_DIR)/radar_bsd.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

radar_bsd_v2: $(SRC_DIR)/radar_bsd_v2.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

radar_bsd_v3: $(SRC_DIR)/radar_bsd_v3.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

radar_bsd_v4: $(SRC_DIR)/radar_bsd_v4.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

baud_scan: $(SRC_DIR)/baud_scan.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

deep_test: $(SRC_DIR)/deep_test.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

baud_fine: $(SRC_DIR)/baud_fine.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

block_test: $(SRC_DIR)/block_test.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

seq_test: $(SRC_DIR)/seq_test.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

time_test: $(SRC_DIR)/time_test.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

clean:
	rm -f $(TARGETS)

deploy: $(TARGETS)
	scp $(TARGETS) root@192.168.88.10:/usr/local/bin/
