
# Change this to whatever AVR programmer you want to use.
PROGRAMMER = usbtiny

# Add _v4 or _v5 for versions >= 4.
OUT=GPS_Clock

# For v3 or previous clocks
CHIP = attiny841
# For v4 or v5 clocks
#CHIP = atxmega32e5

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude -B 0.1
OPTS = -Os -g -std=c11 -Wall -Wno-main

CFLAGS = -mmcu=$(CHIP) $(OPTS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	$(OUT).hex $(OUT).hex

clean:
	rm -f *.hex *.elf *.o

flash:	$(OUT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:$(OUT).hex

fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xd5:m -U lfuse:w:0xe2:m -U efuse:w:0xff:m

init:	fuse flash
