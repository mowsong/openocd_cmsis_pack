BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= arm-none-eabi-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS =	-c -mthumb -mcpu=cortex-m0plus -Os -Wall -Wextra -Wpedantic \
		-fpic -mpic-data-is-text-relative -ffreestanding

all: psoc4hv_write.inc

.PHONY: clean

.INTERMEDIATE: psoc4hv_write.o

psoc4hv_write.o: psoc4hv_write.c Makefile
	$(CC) $(CFLAGS) -DPSOC4_VARIANT=PSOC4_VARIANT_LEGACY -Wa,-adhln=$*.lst $< -o $@

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.o *.lst *.bin *.inc
