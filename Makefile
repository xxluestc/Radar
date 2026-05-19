CROSS_COMPILE = aarch64-none-linux-gnu-
CC = $(CROSS_COMPILE)gcc
STRIP = $(CROSS_COMPILE)strip

TOOLCHAIN_PATH = /home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin
export PATH := $(TOOLCHAIN_PATH):$(PATH)

CFLAGS = -Wall -O2 -static

SRC_DIR = src
TARGETS = radar_bsd

.PHONY: all clean deploy

all: $(TARGETS)

radar_bsd: $(SRC_DIR)/radar_bsd.c
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

clean:
	rm -f $(TARGETS)

deploy: $(TARGETS)
	scp $(TARGETS) root@192.168.88.10:/usr/local/bin/
