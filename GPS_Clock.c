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

// Fuse settings: lfuse=0xe2, hfuse=0xdf, efuse=0xff

// V2 hardware has the PPS going into the ICP pin instead of INT0
#define V2

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

// 8 MHz.
#define F_CPU (8000000UL)
#include <util/delay.h>

// UBRR?_VALUE macros defined here are used below in serial initialization in main()
#define BAUD 9600
#include <util/setbaud.h>

// Port A is used for the display SPI interface and serial.
// We don't need to config the serial pins - it's enough to
// just turn on the USART.
#define PORT_MAX PORTA
#define BIT_MAX_CS _BV(PORTA3)
#define BIT_MAX_CLK _BV(PORTA4)
#define BIT_MAX_DO _BV(PORTA6)
#define DDR_BITS_A _BV(DDRA3) | _BV(DDRA4) | _BV(DDRA6)

// The MAX6951 registers and their bits
#define MAX_REG_DEC_MODE 0x01
#define MAX_REG_INTENSITY 0x02
#define MAX_REG_SCAN_LIMIT 0x03
#define MAX_REG_CONFIG 0x04
#define MAX_REG_CONFIG_R _BV(5)
#define MAX_REG_CONFIG_T _BV(4)
#define MAX_REG_CONFIG_E _BV(3)
#define MAX_REG_CONFIG_B _BV(2)
#define MAX_REG_CONFIG_S _BV(0)
#define MAX_REG_TEST 0x07
// P0 and P1 are planes - used when blinking is turned on
// or the mask with the digit number 0-7. On the hardware, 0-6
// are the digits from left to right (D6 is tenths of seconds, D0
// is tens of hours). D7 is AM, PM and the four LEDs for the colons.
// To blink, you write different stuff to P1 and P0 and turn on
// blinking in the config register (bit E to turn on, bit B for speed).
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

// Digit 7 has the two colons and the AM & PM lights
#define MASK_COLON_HM (MASK_E | MASK_F)
#define MASK_COLON_MS (MASK_B | MASK_C)
#define MASK_AM (MASK_A)
#define MASK_PM (MASK_D)

// Digit map
#define DIGIT_10_HR (0)
#define DIGIT_1_HR (1)
#define DIGIT_10_MIN (2)
#define DIGIT_1_MIN (3)
#define DIGIT_10_SEC (4)
#define DIGIT_1_SEC (5)
#define DIGIT_100_MSEC (6)
#define DIGIT_MISC (7)

#define RX_BUF_LEN (96)

// Port B is the switches and the PPS GPS input
#define PORT_SW PINB
#define DDR_BITS_B (0)
#define SW_0_BIT _BV(PINB0)
#define SW_1_BIT _BV(PINB2)
// Note that some versions of the AVR LIBC forgot to
// define the individual PUExn bit numbers. If you have
// a version like this, then just use _BV(0) | _BV(2).
#define PULLUP_BITS_B _BV(PUEB0) | _BV(PUEB2)

// These are return values from the DST detector routine.
// DST is not in effect all day
#define DST_NO 0
// DST is in effect all day
#define DST_YES 1
// DST begins at 0200
#define DST_BEGINS 2
// DST ends 0300 - that is, at 0200 pre-correction.
#define DST_ENDS 3

// The possible values for dst_mode
#define DST_OFF 0
#define DST_US 1
#define DST_EU 2
#define DST_AU 3
#define DST_NZ 4
#define DST_MODE_MAX DST_NZ

#define COLON_OFF 0
#define COLON_ON 1
#define COLON_BLINK 2
#define COLON_STATE_MAX COLON_BLINK

// EEPROM locations to store the configuration.
#define EE_TIMEZONE ((uint8_t*)0)
#define EE_DST_MODE ((uint8_t*)1)
#define EE_AM_PM ((uint8_t*)2)
#define EE_BRIGHTNESS ((uint8_t*)3)
#define EE_TENTHS ((uint8_t*)4)
#define EE_COLONS ((uint8_t*)5)

// This is the timer frequency - it's the system clock prescaled by 1024
#define F_TICK (F_CPU / 1024)

// We want something like 50 ms.
#define DEBOUNCE_TICKS (F_TICK / 20)
// The buttons
#define SELECT 1
#define SET 2

// If we don't get a PPS at least this often, then we've lost it.
// This is F_TICK*1.25 - a quarter second late.
#define LOST_PPS_TICKS (F_TICK + F_TICK / 4)

// For unknown reasons, we sometimes get a first PPS tick that's way, way
// too fast. Rather than have the display look weird, we'll just skip
// showing tenths anytime GPS tells us a tenth of a second is less than
// 50 ms worth of system clock.
#define FAST_PPS_TICKS (F_TICK / 20)

volatile unsigned char disp_buf[8];
volatile unsigned char rx_buf[RX_BUF_LEN];
volatile unsigned char rx_str_len;
volatile unsigned int last_pps_tick;
volatile unsigned int tenth_ticks;
volatile unsigned char gps_locked;
volatile unsigned char dst_mode;
volatile unsigned char ampm;
volatile unsigned char menu_pos;
volatile char tz_hour;
volatile unsigned char tenth_enable;
volatile unsigned char disp_tenth;
unsigned char colon_state;
unsigned int debounce_time;
unsigned char button_down;
unsigned char brightness;

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

void write_reg(const unsigned char addr, const unsigned char val) {
	// We write a 16 bit word, with address at the top end.
	unsigned int data = (addr << 8) | val;

	// ISRs can perform writes, so the whole thing has to be atomic.
	// A collision is unlikely, but it would be messy.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// Start with clk low
		PORT_MAX &= ~BIT_MAX_CLK;
		// Now assert !CS
		PORT_MAX &= ~BIT_MAX_CS;
		// now clock each data bit in
		for(unsigned int mask = 1 << 15; mask != 0; mask >>= 1) {
			// Set the data bit to the appropriate data bit value
			if (data & mask)
				PORT_MAX |= BIT_MAX_DO;
			else
				PORT_MAX &= ~BIT_MAX_DO;
			// Toggle the clock. The maximum clock frequency is something
			// like 50 MHz, so there's no need to add any delays.
			PORT_MAX |= BIT_MAX_CLK;
			PORT_MAX &= ~BIT_MAX_CLK;
		}
		// And finally, release !CS.
		PORT_MAX |= BIT_MAX_CS;
	}
}

// zero-based day of year number for first day of each month, non-leap-year
const unsigned int first_day[] PROGMEM = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

// Note that this routine is only defined for years between 2000 and 2099.
// If you're alive beyond that, then this is a Y2.1K bug for you to fix.
// You also get to figure out what to do about the NMEA two-digit
// year format. And probably update the GPS firmware or buy a new module.
static unsigned char first_sunday(const unsigned char m, const unsigned char y) {
	unsigned int day_of_year = 0;
	// after March, leap year comes into play
	if (m > 2) {
		// Y2.1K bug here. Every 100 years is not a leap year,
		// but every 400 years is again.
		if ((y % 4) == 0) day_of_year++;
	}
	// Now add in the offset for the month.
	day_of_year += pgm_read_dword(&(first_day[m - 1]));
	// This requires some explanation. We need to calculate the weekday of January 1st.
	// This all works because Y2K was a leap year and because we're doing all
	// of this for years *after* 2000.
	//
	// January 1st 2000 was a Saturday. Since we're 0 based and starting with Sunday...
	unsigned char weekday = 6;
	// For every year after 2000, add one day, since normal years advance the weekday by 1.
	weekday += y % 7;
	weekday %= 7;
	// For every leap year after 2000, add another day, since leap years advance the weekday by 2.
	// But you only do it for leap years *in the past*. Note that this won't work for 2000 because
	// of underflow.
	weekday += (((y - 1) / 4) + 1) % 7;
	weekday %= 7;
	// Y2.1K bug - this is where you're supposed to take a day *away*
	// every 100 years... but then add it *back* every 400.
	// Add in the day of the year
	weekday += day_of_year % 7;
	weekday %= 7;
	// Now figure out how many days before we hit a Sunday. But if we're already there, then just return 1.
	return (weekday == 0)?1:(8 - weekday);
}
static unsigned char calculateDSTAU(const unsigned char d, const unsigned char m, const unsigned char y) {
        // DST is in effect between the first Sunday in October and the first Sunday in April
        unsigned char change_day;
        switch(m) {
                case 1: // November through March
                case 2:
                case 3:
                case 11:
                case 12:
                        return DST_YES;
                case 4: // April
                        change_day = first_sunday(m, y);
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                case 5: // April through September
                case 6:
                case 7:
                case 8:
                case 9:
                        return DST_NO;
                case 10: // October
                        change_day = first_sunday(m, y);
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static unsigned char calculateDSTNZ(const unsigned char d, const unsigned char m, const unsigned char y) {
        // DST is in effect between the last Sunday in September and the first Sunday in April
        unsigned char change_day;
        switch(m) {
                case 1: // October through March
                case 2:
                case 3:
                case 10:
                case 11:
                case 12:
                        return DST_YES;
                case 4: // April
                        change_day = first_sunday(m, y);
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                case 5: // April through August
                case 6:
                case 7:
                case 8:
                        return DST_NO;
                case 9: // September
                        change_day = first_sunday(m, y);
                        while(change_day + 7 < 30) change_day += 7; // last Sunday
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static unsigned char calculateDSTEU(const unsigned char d, const unsigned char m, const unsigned char y) {
        // DST is in effect between the last Sunday in March and the last Sunday in October
        unsigned char change_day;
        switch(m) {
                case 1: // November through February
                case 2:
                case 11:
                case 12:
                        return DST_NO;
                case 3: // March
                        change_day = first_sunday(m, y);
                        while(change_day + 7 < 31) change_day += 7; // last Sunday
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                case 4: // April through September
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                        return DST_YES;
                case 10: // October
                        change_day = first_sunday(m, y);
                        while(change_day + 7 < 30) change_day += 7; // last Sunday
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static unsigned char calculateDSTUS(const unsigned char d, const unsigned char m, const unsigned char y) {
	// DST is in effect between the 2nd Sunday in March and the first Sunday in November
	// The return values here are that DST is in effect, or it isn't, or it's beginning
	// for the year today or it's ending today.
	unsigned char change_day;
	switch(m) {
		case 1: // December through February
		case 2:
		case 12:
			return DST_NO;
		case 3: // March
			change_day = first_sunday(m, y) + 7; // second Sunday.
			if (d < change_day) return DST_NO;
			else if (d == change_day) return DST_BEGINS;
			else return DST_YES;
			break;
		case 4: // April through October
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			return DST_YES;
		case 11: // November
			change_day = first_sunday(m, y);
			if (d < change_day) return DST_YES;
			else if (d == change_day) return DST_ENDS;
			else return DST_NO;
			break;
		default: // This is impossible, since m can only be between 1 and 12.
			return 255;
	}
}
static unsigned char calculateDST(const unsigned char d, const unsigned char m, const unsigned char y) {
        switch(dst_mode) {
                case DST_US:
                        return calculateDSTUS(d, m, y);
                case DST_EU:
                        return calculateDSTEU(d, m, y);
                case DST_AU:
                        return calculateDSTAU(d, m, y);
                case DST_NZ:
                        return calculateDSTNZ(d, m, y);
                default: // off - should never happen
                        return DST_NO;
        }
}

static void handle_time(char h, unsigned char m, unsigned char s, unsigned char dst_flags) {
	// What we get is the current second. We have to increment it
	// to represent the *next* second.
	s++;
	// Note that this also handles leap-seconds. We wind up pinning to 0
	// twice.
	if (s >= 60) { s = 0; m++; }
	if (m >= 60) { m = 0; h++; }
	if (h >= 24) { h = 0; }

	// Move to local standard time.
	h += tz_hour;
	while (h >= 24) h -= 24;
	while (h < 0) h += 24;

	if (dst_mode != DST_OFF) {
		unsigned char dst_offset = 0;
		switch(dst_flags) {
			case DST_NO: dst_offset = 0; break; // do nothing
			case DST_YES: dst_offset = 1; break; // add one hour
			case DST_BEGINS:
				dst_offset = (h >= (dst_mode == DST_EU?1:2))?1:0; break; // offset becomes 1 at 0200 (0100 EU)
                        case DST_ENDS:
                                dst_offset = (h >= (dst_mode == DST_EU?0:1))?0:1; break; // offset becomes 0 at 0200 (post-correction) (0100 EU)
		}
		h += dst_offset;
		if (h >= 24) h -= 24;
	}

	unsigned char am = 0;
	if (ampm) {
		// Create AM or PM
                if (h == 0) { h = 12; am = 1; }
		else if (h < 12) { am = 1; }
		else if (h > 12) h -= 12;
	}

	disp_buf[DIGIT_1_SEC] = (s % 10) | MASK_DP;
	disp_buf[DIGIT_10_SEC] = s / 10;
	disp_buf[DIGIT_1_MIN] = (m % 10);
	disp_buf[DIGIT_10_MIN] = m / 10;
	disp_buf[DIGIT_1_HR] = (h % 10);
	disp_buf[DIGIT_10_HR] = h / 10;
	disp_buf[DIGIT_100_MSEC] = disp_buf[DIGIT_MISC] = 0;
	if (ampm) {
#if 1
		disp_buf[DIGIT_MISC] |= am ? MASK_AM : MASK_PM;
#else
// Early hardware used the digit 6 and 7 DPs for AM/PM.
		if (am)
			disp_buf[7] |= MASK_DP;
		else
			disp_buf[6] |= MASK_DP;
#endif
	}
	if (colon_state == COLON_ON || colon_state == COLON_BLINK) {
		disp_buf[DIGIT_MISC] |= MASK_COLON_HM | MASK_COLON_MS;
	}
}

static const char *skip_commas(const char *ptr, const int num) {
	for(int i = 0; i < num; i++) {
		ptr = strchr(ptr, ',');
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
  
	const char *ptr = (char *)rx_buf;
	if (!strncmp_P(ptr, PSTR("$GPRMC"), 6)) {
		// $GPRMC,172313.000,A,xxxx.xxxx,N,xxxxx.xxxx,W,0.01,180.80,260516,,,D*74\x0d\x0a
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		char h = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char min = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned char s = (ptr[4] - '0') * 10 + (ptr[5] - '0');
		unsigned char dst_flags = 0;
		ptr = skip_commas(ptr, 8);
		if (ptr == NULL) return; // not enough commas
		unsigned char d = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char mon = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned char y = (ptr[4] - '0') * 10 + (ptr[5] - '0');
		// The problem is that our D/M/Y is UTC, but DST decisions are made in the local
		// timezone. We can adjust the day against standard time midnight, and
		// that will be good enough. Don't worry that this can result in d being either 0
		// or past the last day of the month. Neither of those will match the "decision day"
		// for DST, which is the only day on which the day of the month is significant.
		if (h + tz_hour < 0) d--;
		if (h + tz_hour > 23) d++;
		dst_flags = calculateDST(d, mon, y);
		handle_time(h, min, s, dst_flags);
	} else if (!strncmp_P(ptr, PSTR("$GPGSA"), 6)) {
		// $GPGSA,A,3,02,06,12,24,25,29,,,,,,,1.61,1.33,0.90*01
		ptr = skip_commas(ptr, 2);
		if (ptr == NULL) return; // not enough commas
		gps_locked = (*ptr == '3' || *ptr == '2');
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
	tenth_ticks = 0;
	// Clear out the digit data
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E);
	write_reg(MAX_REG_DEC_MODE, 0);
        write_reg(MAX_REG_MASK_BOTH | 0, MASK_C | MASK_E | MASK_G); // n
        write_reg(MAX_REG_MASK_BOTH | 1, MASK_C | MASK_D | MASK_E | MASK_G); // o
        write_reg(MAX_REG_MASK_BOTH | 3, MASK_A | MASK_C | MASK_D | MASK_E | MASK_F | MASK_G); // G
        write_reg(MAX_REG_MASK_BOTH | 4, MASK_A | MASK_B | MASK_E | MASK_F | MASK_G); // P
        write_reg(MAX_REG_MASK_BOTH | 5, MASK_A | MASK_C | MASK_D | MASK_F | MASK_G); // S
}

#ifdef V2
ISR(TIMER1_CAPT_vect) {
#else
ISR(INT0_vect) {
#endif
#ifdef V2
	unsigned int this_tick = ICR1;
#else
	unsigned int this_tick = TCNT1;
#endif
	if (menu_pos == 0 && tenth_enable && last_pps_tick != 0) {
		tenth_ticks = (this_tick - last_pps_tick) / 10;
		// For unknown reasons we seemingly sometimes get spurious
		// PPS interrupts. If the calculus leads us to believe a
		// a tenth of a second is less than 50 ms worth of system clock,
		// then it's not right - just skip it.
		if (tenth_ticks < FAST_PPS_TICKS) tenth_ticks = 0;
	} else {
		tenth_ticks = 0;
	}
	last_pps_tick = this_tick;
	if (last_pps_tick == 0) last_pps_tick++; // it can never be zero

	if (menu_pos) return;
	if (!gps_locked) {
		write_no_sig();
		return;
	}

	// Hit the T bit so that the blink counter is cleared
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E | MAX_REG_CONFIG_T);
        unsigned char decode_mask = 0x7f; // assume decoding for all digits
	// If we are doing 12 digit display and if the 10 hours digit is 0, then blank it instead.
	// Its value will be zero, so simply disabling the hex decode will result in no segments.
	if (ampm && disp_buf[DIGIT_10_HR] == 0) {
		decode_mask &= ~_BV(DIGIT_10_HR); // No decode for tens-of-hours digit
	}
	// If we're not going to show the tenths...
	if (tenth_ticks == 0) {
		decode_mask &= ~_BV(DIGIT_100_MSEC); // No decode for tenth digit
		disp_buf[DIGIT_1_SEC] &= ~MASK_DP; // no decimal point on seconds digit
	}
	write_reg(MAX_REG_DEC_MODE, decode_mask);

	disp_tenth = 0; // right now, 0 is showing.

	// Copy the display buffer data into the display
	for(int i = 0; i < sizeof(disp_buf); i++) {
		write_reg(MAX_REG_MASK_BOTH | i, disp_buf[i]);
	}
	if (colon_state == COLON_BLINK) {
		// If we're blinking, then clear P1's colons out.
		write_reg(MAX_REG_MASK_P1 | DIGIT_MISC, disp_buf[DIGIT_MISC] & ~(MASK_COLON_HM | MASK_COLON_MS));
	}
}

static unsigned char check_buttons() {
	// TCNT1 is a 16 bit register - it must be read atomically.
	unsigned int tcnt1;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		tcnt1 = TCNT1;
	}
	if (debounce_time != 0 && tcnt1 - debounce_time < DEBOUNCE_TICKS) {
		// We don't pay any attention to the buttons during debounce time.
		return 0;
	} else {
		debounce_time = 0; // debounce is over
	}
	unsigned char status = PORT_SW & (SW_0_BIT | SW_1_BIT);
	status ^= (SW_0_BIT | SW_1_BIT); // invert the buttons - 0 means down.
	if (!((button_down == 0) ^ (status == 0))) return 0; // either no button is down, or a button is still down

	// Something *changed*, which means we must now start a debounce interval.
	debounce_time = tcnt1;
	if (!debounce_time) debounce_time++; // it's not allowed to be zero

	if (!button_down && status) {
		button_down = 1; // a button has been pushed
		return (status & SW_1_BIT)?SELECT:SET;
	}
	if (button_down && !status) {
		button_down = 0; // a button has been released
		return 0;
	}
	__builtin_unreachable(); // we'll never get here.
}

static void menu_render() {
	// blank the display
	write_reg(MAX_REG_DEC_MODE, 0); // no decoding
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E);
	switch(menu_pos) {
		case 0:
			// we're returning to time mode. Either leave it blank or indicate no signal.
			if (!gps_locked)
				write_no_sig();
			tenth_ticks = 0;
			break;
		case 1: // zone
			write_reg(MAX_REG_DEC_MODE, 0x30); // decoding for last two digits only
        		write_reg(MAX_REG_MASK_BOTH | 0, MASK_D | MASK_E | MASK_F | MASK_G); // t
        		write_reg(MAX_REG_MASK_BOTH | 1, MASK_C | MASK_E | MASK_F | MASK_G); // h
			if (tz_hour < 0) {
        			write_reg(MAX_REG_MASK_BOTH | 3, MASK_G); // -
			}
        		write_reg(MAX_REG_MASK_BOTH | 4, abs(tz_hour) / 10);
        		write_reg(MAX_REG_MASK_BOTH | 5, abs(tz_hour) % 10);
			break;
		case 2: // DST on/off
			write_reg(MAX_REG_DEC_MODE, 0); // no decoding
        		write_reg(MAX_REG_MASK_BOTH | 0, MASK_B | MASK_C | MASK_D | MASK_E | MASK_G); // d
        		write_reg(MAX_REG_MASK_BOTH | 1, MASK_A | MASK_C | MASK_D | MASK_F | MASK_G); // S
			switch(dst_mode) {
                                case DST_OFF:
                                        write_reg(MAX_REG_MASK_BOTH | 3, MASK_C | MASK_D | MASK_E | MASK_G); // o
                                        write_reg(MAX_REG_MASK_BOTH | 4, MASK_A | MASK_E | MASK_F | MASK_G); // F
                                        write_reg(MAX_REG_MASK_BOTH | 5, MASK_A | MASK_E | MASK_F | MASK_G); // F
                                        break;
                                case DST_EU:
                                        write_reg(MAX_REG_MASK_BOTH | 3, MASK_A | MASK_D | MASK_E | MASK_F | MASK_G); // E
                                        write_reg(MAX_REG_MASK_BOTH | 4, MASK_B | MASK_C | MASK_D | MASK_E | MASK_F); // U
                                        break;
                                case DST_US:
                                        write_reg(MAX_REG_MASK_BOTH | 3, MASK_B | MASK_C | MASK_D | MASK_E | MASK_F); // U
                                        write_reg(MAX_REG_MASK_BOTH | 4, MASK_A | MASK_C | MASK_D | MASK_F | MASK_G); // S
                                        break;
                                case DST_AU:
                                        write_reg(MAX_REG_MASK_BOTH | 3, MASK_A | MASK_B | MASK_C | MASK_E | MASK_F | MASK_G); // A
                                        write_reg(MAX_REG_MASK_BOTH | 4, MASK_B | MASK_C | MASK_D | MASK_E | MASK_F); // U
                                        break;
                                case DST_NZ:
                                        write_reg(MAX_REG_MASK_BOTH | 3, MASK_C | MASK_E | MASK_G); // n
                                        write_reg(MAX_REG_MASK_BOTH | 4, MASK_A | MASK_B | MASK_D | MASK_E | MASK_G); // Z
                                        break;
                        }
			break;
		case 3: // 12/24 hour
			write_reg(MAX_REG_DEC_MODE, 0x6); // decoding for first two digits only (skipping one)
        		write_reg(MAX_REG_MASK_BOTH | 1, ampm?1:2);
        		write_reg(MAX_REG_MASK_BOTH | 2, ampm?2:4);
        		write_reg(MAX_REG_MASK_BOTH | 4, MASK_C | MASK_E | MASK_F | MASK_G); // h
        		write_reg(MAX_REG_MASK_BOTH | 5, MASK_E | MASK_G); // r
			break;
		case 4: // tenths enabled
			write_reg(MAX_REG_DEC_MODE, 3); // decode only first two digits
        		write_reg(MAX_REG_MASK_BOTH | 0, 1);
        		write_reg(MAX_REG_MASK_BOTH | 1, 0);
			if (tenth_enable) {
				write_reg(MAX_REG_MASK_BOTH | 3, MASK_C | MASK_D | MASK_E | MASK_G); // o
				write_reg(MAX_REG_MASK_BOTH | 4, MASK_C | MASK_E | MASK_G); // n
			} else {
				write_reg(MAX_REG_MASK_BOTH | 3, MASK_C | MASK_D | MASK_E | MASK_G); // o
				write_reg(MAX_REG_MASK_BOTH | 4, MASK_A | MASK_E | MASK_F | MASK_G); // F
				write_reg(MAX_REG_MASK_BOTH | 5, MASK_A | MASK_E | MASK_F | MASK_G); // F
			}
			break;
		case 5: // colons enabled
			write_reg(MAX_REG_DEC_MODE, 0); // decode only first two digits
        		write_reg(MAX_REG_MASK_BOTH | 0, MASK_A | MASK_D | MASK_E | MASK_F); // C
        		write_reg(MAX_REG_MASK_BOTH | 1, MASK_C | MASK_D | MASK_E | MASK_G); // o
        		write_reg(MAX_REG_MASK_BOTH | 2, MASK_D | MASK_E | MASK_F); // L
        		write_reg(MAX_REG_MASK_BOTH | 3, MASK_C | MASK_D | MASK_E | MASK_G); // o
			write_reg(MAX_REG_MASK_BOTH | 4, MASK_C | MASK_E | MASK_G); // n
			write_reg(MAX_REG_MASK_BOTH | 5, MASK_A | MASK_C | MASK_D | MASK_F | MASK_G); // S
			switch(colon_state) {
				case COLON_OFF: // nothing
					break;
				case COLON_ON: // on solid
					write_reg(MAX_REG_MASK_BOTH | 7, MASK_COLON_HM | MASK_COLON_MS);
					break;
				case COLON_BLINK: // blink - write to only P0
					write_reg(MAX_REG_MASK_P0 | 7, MASK_COLON_HM | MASK_COLON_MS);
					break;
			}
			break;
		case 6: // brightness
			write_reg(MAX_REG_DEC_MODE, 0); // no decoding
        		write_reg(MAX_REG_MASK_BOTH | 0, MASK_C | MASK_D | MASK_E | MASK_F | MASK_G); // b
        		write_reg(MAX_REG_MASK_BOTH | 1, MASK_E | MASK_G); // r
        		write_reg(MAX_REG_MASK_BOTH | 2, MASK_B | MASK_C); // I
        		write_reg(MAX_REG_MASK_BOTH | 3, MASK_A | MASK_C | MASK_D | MASK_E | MASK_F | MASK_G); // G
        		write_reg(MAX_REG_MASK_BOTH | 4, MASK_C | MASK_E | MASK_F | MASK_G); // h
        		write_reg(MAX_REG_MASK_BOTH | 5, MASK_D | MASK_E | MASK_F | MASK_G); // t
			write_reg(MAX_REG_INTENSITY, brightness);
			break;
	}
}

static void menu_set() {
	switch(menu_pos) {
		case 0:
			tenth_ticks = 0;
			break;
		case 1:
			eeprom_write_byte(EE_TIMEZONE, tz_hour + 12);
			break;
		case 2:
			eeprom_write_byte(EE_DST_MODE, dst_mode);
			break;
		case 3:
			eeprom_write_byte(EE_AM_PM, ampm);
			break;
		case 4:
			eeprom_write_byte(EE_TENTHS, tenth_enable);
			break;
		case 5:
			eeprom_write_byte(EE_COLONS, colon_state);
			break;
		case 6:
			eeprom_write_byte(EE_BRIGHTNESS, brightness);
			break;
	}
	if (++menu_pos > 6) menu_pos = 0;
	menu_render();
}

static void menu_select() {
	switch(menu_pos) {
		case 0: return; // ignore SET when just running
		case 1: // timezone
			if (++tz_hour >= 13) tz_hour = -12;
			break;
		case 2: // DST on/off
			if (++dst_mode > DST_MODE_MAX) dst_mode = 0;
			break;
		case 3: // 12/24 hour
			ampm = !ampm;
			break;
		case 4: // tenths enabled
			tenth_enable = !tenth_enable;
			break;
		case 5: // colons
			if (++colon_state > COLON_STATE_MAX) colon_state = 0;
			break;
		case 6: // brightness
			if (++brightness > 15) brightness = 0;
			break;
	}
	menu_render();
}

// main() never returns.
void __ATTR_NORETURN__ main(void) {

	wdt_enable(WDTO_1S);

	// We don't use a lot of the chip, it turns out
	PRR = ~(_BV(PRUSART0) | _BV(PRTIM1));

	// Make sure the CS pin is high and everything else is low.
	PORT_MAX = BIT_MAX_CS;
	DDRA = DDR_BITS_A;

	DDRB = DDR_BITS_B;
	PUEB = PULLUP_BITS_B;

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

#ifndef V2
	MCUCR = _BV(ISC01) | _BV(ISC00); // INT0 on rising edge
	GIMSK = _BV(INT0); // enable INT0
#endif

	rx_str_len = 0;

#ifdef V2
	TCCR1B = _BV(ICES1) | _BV(CS12) | _BV(CS10); // prescale by 1024, capture on rising edge
	TIMSK1 = _BV(ICIE1); // interrupt on capture
#else
	TCCR1B = _BV(CS12) | _BV(CS10); // prescale by 1024
#endif

	unsigned char ee_rd = eeprom_read_byte(EE_TIMEZONE);
	if (ee_rd == 0xff)
		tz_hour = -8;
	else
		tz_hour = ee_rd - 12;
	dst_mode = eeprom_read_byte(EE_DST_MODE);
	if (dst_mode > DST_MODE_MAX) dst_mode = DST_US;
	ampm = eeprom_read_byte(EE_AM_PM) != 0;

	gps_locked = 0;
	menu_pos = 0;
	debounce_time = 0;
	button_down = 0;
	last_pps_tick = 0;
	tenth_ticks = 0;
	disp_tenth = 0;

	// Turn off the shut-down register, clear the digit data
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E);
	write_reg(MAX_REG_SCAN_LIMIT, 7); // display all 8 digits
	brightness = eeprom_read_byte(EE_BRIGHTNESS) & 0xf;
	write_reg(MAX_REG_INTENSITY, brightness);

	tenth_enable = eeprom_read_byte(EE_TENTHS) != 0;
        colon_state = eeprom_read_byte(EE_COLONS);
	if (colon_state > COLON_STATE_MAX) colon_state = 1; // default to just on.

	// Turn on the self-test for a second
	write_reg(MAX_REG_TEST, 1);
	Delay(1000);
	write_reg(MAX_REG_TEST, 0);
	write_no_sig();

	// Turn on interrupts
	sei();

	while(1) {
		wdt_reset();
		unsigned int local_lpt, tcnt1;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			local_lpt = last_pps_tick;
			tcnt1 = TCNT1;
		}
		// If we've not seen a PPS pulse in a certain amount of time, then
		// without doing something like this, the wrong time would just get stuck.
		if (local_lpt != 0 && tcnt1 - local_lpt > LOST_PPS_TICKS) {
			write_no_sig();
			last_pps_tick = 0;
			continue;
		}
		if (tenth_ticks != 0) {
			unsigned int current_tick = tcnt1 - local_lpt;
			unsigned int current_tenth = (current_tick / tenth_ticks) % 10;
			if (disp_tenth != current_tenth) {
				// Write the tenth-of-a-second digit, preserving the
				// decimal point state (just in case)
				write_reg(MAX_REG_MASK_BOTH | DIGIT_100_MSEC, current_tenth | (disp_buf[DIGIT_100_MSEC] & MASK_DP));
				disp_tenth = current_tenth;
			}
		}
		unsigned char button = check_buttons();
		if (!button) continue;

		switch(button) {
			case SELECT:
				menu_select();
				break;
			case SET:
				menu_set();
				break;
		}
	}
	__builtin_unreachable();
}
