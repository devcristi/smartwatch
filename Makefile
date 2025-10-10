# Makefile for STM-32 Smartwatch Project

CC=arm-none-eabi-gcc
CFLAGS=-mcpu=cortex-m4 -mthumb -Wall -g
LDFLAGS=-Tlinker_script.ld

SRCS=$(wildcard Core/Src/*.c Drivers/Custom/*.c)
OBJS=$(SRCS:.c=.o)

all: main.elf

main.elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)

clean:
	rm -f $(OBJS) main.elf