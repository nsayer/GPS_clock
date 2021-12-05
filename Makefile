

# For v3 or previous clocks
#OUT=GPS_Clock
#CHIP = attiny841
#PROGRAMMER = usbtiny -B 5
# For v5 clocks
OUT=GPS_Clock_v5
CHIP = atxmega32e5
PROGRAMMER = atmelice_pdi

CC = avr-gcc
OBJCPY = avr-objcopy
AVRDUDE = avrdude
OPTS = -Os -g -std=c11 -Wall -Wno-main -fno-tree-switch-conversion

CFLAGS = -mmcu=$(CHIP) $(OPTS)

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.hex: %.elf
	$(OBJCPY) -j .text -j .data -O ihex $^ $@

%.elf: %.o
	$(CC) $(CFLAGS) -o $@ $^

all:	$(OUT).hex

clean:
	rm -f *.hex *.elf *.o

flash:	$(OUT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U flash:w:$(OUT).hex

# note that the fuse target is for ATTiny841 (version < 4) only.
fuse:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(CHIP) -U hfuse:w:0xd5:m -U lfuse:w:0xe2:m -U efuse:w:0xff:m

init:	fuse flash
