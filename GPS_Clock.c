/*

    GPS Clock
    Copyright (C) 2016 Nicholas W. Sayer

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
    
  */

// If you want a 12 hour clock with AM/PM, uncomment this
// For the Hackaday 1K contest, this has to be left off,
// and timezone support can't be added.
// #define AMPM 1

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

// 10 MHz.
#define F_CPU (10000000UL)
#include <util/delay.h>

// UBRR?_VALUE macros defined here are used below in serial initialization in main()
#define BAUD 9600
#include <util/setbaud.h>

#define PORT_MAX PORTA
#define BIT_MAX_CS _BV(PORTA3)
#define BIT_MAX_CLK _BV(PORTA4)
#define BIT_MAX_DO _BV(PORTA6)
#define DDR_BITS_A _BV(DDRA3) | _BV(DDRA4) | _BV(DDRA6)

#define PORT_SW PINB
#define DDR_BITS_B (0)
#define SW_0_BIT _BV(PINB0)
#define SW_1_BIT _BV(PINB2)
#define PULLUP_BITS_B _BV(PUEB0) | _BV(PUEB2)

#define MAX_REG_DEC_MODE 0x01
#define MAX_REG_INTENSITY 0x02
#define MAX_REG_SCAN_LIMIT 0x03
#define MAX_REG_CONFIG 0x04
#define MAX_REG_CONFIG_R _BV(5)
#define MAX_REG_CONFIG_T _BV(4)
#define MAX_REG_CONFIG_E _BV(3)
#define MAX_REG_CONFIG_B _BV(2)
#define MAX_REG_CONFIG_S _BV(0)
#define MAX_REG_TEST 0x05
// P0 and P1 are planes - used when blinking is turned on
// or the mask with the digit number 0-7. On the hardware, 0-5
// are the digits from right to left (D0 is single seconds, D5
// is tens of hours). The D6 and D7 decimal points are AM and PM.
#define MAX_REG_MASK_P0 0x20
#define MAX_REG_MASK_P1 0x40
#define MAX_REG_MASK_BOTH (MAX_REG_MASK_P0 | MAX_REG_MASK_P1)
// When decoding is turned off, this is the bit mapping.
// Segment A is at the top, the rest proceed clockwise around, and
// G is in the middle. DP is the decimal point.
// When decoding is turned on, bits 0-3 are a hex value, 4-6 are ignored,
// and DP is as before.
#define MASK_DP _BV(7)
#define MASK_A _BV(6)
#define MASK_B _BV(5)
#define MASK_C _BV(4)
#define MASK_D _BV(3)
#define MASK_E _BV(2)
#define MASK_F _BV(1)
#define MASK_G _BV(0)

#define RX_BUF_LEN (96)

volatile unsigned char disp_buf[8];
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;

// Delay, but pet the watchdog while doing it.
static void Delay(unsigned long ms) {
  while(ms > 100) {
    _delay_ms(100);
    wdt_reset();
    ms -= 100;
  }
  _delay_ms(ms);
  wdt_reset();
}

static void write_reg(unsigned char addr, unsigned char val) {
	unsigned int data = (addr << 8) | val;

	// Start with clk high
	PORT_MAX |= BIT_MAX_CLK;
	// Now drop !CS
	PORT_MAX &= ~BIT_MAX_CS;
	// now clock each data bit in
	for(int i = 15; i >= 0; i--) {
		// Set the data bit to the appropriate data bit value
		if ((data >> i) & 0x1)
			PORT_MAX |= BIT_MAX_DO;
		else
			PORT_MAX &= ~BIT_MAX_DO;
		// Toggle the clock
		PORT_MAX &= ~BIT_MAX_CS;
		PORT_MAX |= BIT_MAX_CS;
	}
	// And finally, raise !CS.
	PORT_MAX |= BIT_MAX_CS;
}

ISR(INT0_vect) {
	write_reg(MAX_REG_DEC_MODE, 0x3f); // full decode for 6 digits.
	// Copy the display buffer data into the display
	for(int i = 0; i < sizeof(disp_buf); i++) {
		write_reg(MAX_REG_MASK_BOTH | i, disp_buf[i]);
	}
}

static void handle_time(unsigned char h, unsigned char m, unsigned char s) {
	// What we get is the current second. We have to increment it
	// to represent the *next* second.
	s++;
	// Note that this also handles leap-seconds. We wind up pinning to 0
	// twice.
	if (s >= 60) { s = 0; m++; }
	if (m >= 60) { m = 0; h++; }
	if (m >= 23) { h = 0; }

	// XXX Correct for time-zone here

#ifdef AMPM
	// Create AM or PM
	unsigned char am = 0;
	if (h < 12) { am = 1; }
	else {
		if (h > 12) h -= 12;
	}
#endif

	disp_buf[0] = s % 10;
	disp_buf[1] = s / 10;
	disp_buf[2] = (m % 10) | MASK_DP;
	disp_buf[3] = m / 10;
	disp_buf[4] = (h % 10) | MASK_DP;
	disp_buf[5] = h / 10;
#ifdef AMPM
	disp_buf[6] = am ? MASK_DP:0;
	disp_buf[7] = (!am) ? MASK_DP:0;
#endif
}

static char* skip_commas(char *ptr, int num) {
	for(int i = 0; i < num; i++) {
		ptr = strchr((const char *)ptr, ',');
		if (ptr == NULL) return NULL; // not enough commas
		ptr++; // skip over it
	}
	return ptr;
}

const char hexes[] PROGMEM = "0123456789abcdef";

static unsigned char hexChar(unsigned char c) {
	if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
	const char* outP = strchr_P(hexes, c);
	if (outP == NULL) return 0;
	return (unsigned char)(outP - hexes);
}

static void handleGPS() {
	unsigned int str_len = rx_str_len; // rx_str_len is where the \0 was written.
 
	if (str_len < 9) return; // No sentence is shorter than $GPGGA*xx
	// First, check the checksum of the sentence
	unsigned char checksum = 0;
	int i;
	for(i = 1; i < str_len; i++) {
		if (rx_buf[i] == '*') break;
		checksum ^= rx_buf[i];
	}
	if (i > str_len - 3) {
		return; // there has to be room for the "*" and checksum.
	}
	i++; // skip the *
	unsigned char sent_checksum = (hexChar(rx_buf[i]) << 4) | hexChar(rx_buf[i + 1]);
	if (sent_checksum != checksum) {
		return; // bad checksum.
	}
  
	char *ptr = (char *)rx_buf;
	if (!strncmp_P((const char*)rx_buf, PSTR("$GPRMC"), 6)) {
		// $GPRMC,172313.000,A,xxxx.xxxx,N,xxxxx.xxxx,W,0.01,180.80,260516,,,D*74\x0d\x0a
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		unsigned char h = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char m = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned char s = (ptr[4] - '0') * 10 + (ptr[5] - '0');
		handle_time(h, m, s);
	}
}

ISR(USART0_RX_vect) {
  unsigned char rx_char = UDR0;
  
  if (rx_str_len == 0 && rx_char != '$') return; // wait for a "$" to start the line.
  rx_buf[rx_str_len] = rx_char;
  if (rx_char == 0x0d || rx_char == 0x0a) {
    rx_buf[rx_str_len] = 0; // null terminate
    handleGPS();
    rx_str_len = 0; // now clear the buffer
    return;
  }
  if (++rx_str_len == RX_BUF_LEN) {
    // The string is too long. Start over.
    rx_str_len = 0;
  }
}

static void write_no_sig() {
	write_reg(MAX_REG_DEC_MODE, 0xff); // No decode, all digits
        write_reg(MAX_REG_MASK_BOTH | 5, MASK_E | MASK_G | MASK_C); // n
        write_reg(MAX_REG_MASK_BOTH | 4, MASK_E | MASK_G | MASK_C | MASK_D); // o
        write_reg(MAX_REG_MASK_BOTH | 2, MASK_A | MASK_F | MASK_G | MASK_C | MASK_D); // S
        write_reg(MAX_REG_MASK_BOTH | 1, MASK_B | MASK_C); // I
        write_reg(MAX_REG_MASK_BOTH | 0, MASK_A | MASK_F | MASK_E | MASK_G | MASK_C | MASK_D); // G
}

void main() {

	// We don't use a lot of the chip, it turns out
	PRR = ~_BV(PRUSART0);

	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A = _BV(U2X0);
#else
	UCSR0A = 0;
#endif

	UCSR0B = _BV(RXCIE0) | _BV(RXEN0); // RX interrupt and RX enable

	// 8N1
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

	MCUCR = _BV(ISC01) | _BV(ISC00); // INT0 on rising edge
	GIMSK = _BV(INT0); // enable INT0

	rx_str_len = 0;

	// Turn off the shut-down register, clear the digit data
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_S);
	write_reg(MAX_REG_SCAN_LIMIT, 7); // display all 8 digits
	write_reg(MAX_REG_INTENSITY, 0xf); // Full intensity
	// Turn on the self-test for a second
	write_reg(MAX_REG_TEST, 1);
	Delay(1000);
	write_reg(MAX_REG_TEST, 0);
	write_no_sig();

	sei();
	// Do nothing. We're entirely interrupt driven.
	while(1);
}
