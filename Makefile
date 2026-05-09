CROSS_COMPILE = aarch64-none-linux-gnu-
CC = $(CROSS_COMPILE)gcc
STRIP = $(CROSS_COMPILE)strip

TOOLCHAIN_PATH = /home/alientek/Phytium_syscode/GCC编译器/gcc-arm-10.2-2020.11-x86_64-aarch64-none-linux-gnu/bin
export PATH := $(TOOLCHAIN_PATH):$(PATH)

CFLAGS = -Wall -O2 -static
TARGET = radar_bsd

SRC_DIR = src
SRCS = $(SRC_DIR)/radar_bsd.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean deploy

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^
	$(STRIP) $@

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

deploy: $(TARGET)
	scp $(TARGET) root@192.168.88.10:/usr/local/bin/
