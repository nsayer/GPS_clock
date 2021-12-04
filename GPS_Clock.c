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

// Hardware options:
// ---
// V2 hardware has the PPS going into the ICP pin instead of INT0
#define V2

// V3 has the PPS line going into ICP2 instead of ICP1 so !SS can be !D_CS
#define V3

// V3.1 has the two buttons moved to port A to make room for the crystal
// on pins B0 & B1.
#define V31

// Define this if your board uses an 12 MHz crystal instead of the RC oscillator
//#define XTAL

// !NEW_AMPM hardware has AM on the digit 7 DP and PM on the digit 6 DP
#define NEW_AMPM

// Older hardware doesn't have a tenth of a second digit
#define TENTH_DIGIT

// Older hardware doesn't have colons.
#define COLONS
// ---

#if (defined(V31) && !defined(V3))
#error V31 requires V3
#endif

#if (defined(XTAL) && !defined(V31))
#error XTAL requires V31
#endif

#if (defined(V3) && !defined(V2))
#error V3 requires V2
#endif

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

#ifdef XTAL
// 12 MHz.
#define F_CPU (12000000UL)
#else
// 8 MHz.
#define F_CPU (8000000UL)
#endif

#include <util/delay.h>

// UBRR?_VALUE macros defined here are used below in serial initialization in main()
#define BAUD 9600
#include <util/setbaud.h>

// Port A is used for the display SPI interface and serial.
// We don't need to config the serial pins - it's enough to
// just turn on the USART.
#define PORT_MAX PORTA
#ifdef V3
#define BIT_MAX_CS _BV(PORTA7)
#else
#define BIT_MAX_CS _BV(PORTA3)
#endif
// MOSI and SCK are taken care of by the SPI stuff, but must still
// have the DDR bits set to make them outputs.
#ifdef V3
#define DDR_BITS_A _BV(DDA7) | _BV(DDA4) | _BV(DDA6)
#else
#define DDR_BITS_A _BV(DDA3) | _BV(DDA4) | _BV(DDA6)
#endif

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
#ifdef COLONS
#define MASK_COLON_HM (MASK_E | MASK_F)
#define MASK_COLON_MS (MASK_B | MASK_C)
#endif
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
#define TX_BUF_LEN (24)

// Note that some versions of the AVR LIBC forgot to
// define the individual PUExn bit numbers.
#ifndef PUEA0
#define PUEA0 0
#define PUEA1 1
#define PUEA2 2
#define PUEA3 3
#define PUEA4 4
#define PUEA5 5
#define PUEA6 6
#define PUEA7 7
#define PUEB0 0
#define PUEB1 1
#define PUEB2 2
#define PUEB3 3
#endif

#ifdef V31
#define PORT_SW PINA
#define SW_0_BIT _BV(PINA0)
#define SW_1_BIT _BV(PINA3)
// DDR_BITS_A doesn't need to be adjusted. The correct bits are already inputs.
// Port B only has the PPS input, so it's zero.
#define DDR_BITS_B (0)
#define PULLUP_BITS_A _BV(PUEA0) | _BV(PUEA3)
#else // !V31
// Port B is the switches and the PPS GPS input
#define PORT_SW PINB
#define DDR_BITS_B (0)
#define SW_0_BIT _BV(PINB0)
#ifdef V3
#define SW_1_BIT _BV(PINB1)
#else // !V3
#define SW_1_BIT _BV(PINB2)
#endif
#ifdef V3
#define PULLUP_BITS_B _BV(PUEB0) | _BV(PUEB1)
#else // !V3
#define PULLUP_BITS_B _BV(PUEB0) | _BV(PUEB2)
#endif
#endif

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

#ifdef COLONS
#define COLON_OFF 0
#define COLON_ON 1
#define COLON_BLINK 2
#define COLON_STATE_MAX COLON_BLINK
#endif

// EEPROM locations to store the configuration.
#define EE_TIMEZONE ((uint8_t*)0)
#define EE_DST_MODE ((uint8_t*)1)
#define EE_AM_PM ((uint8_t*)2)
#define EE_BRIGHTNESS ((uint8_t*)3)
#ifdef TENTH_DIGIT
#define EE_TENTHS ((uint8_t*)4)
#endif
#ifdef COLONS
#define EE_COLONS ((uint8_t*)5)
#endif

// This is the timer frequency - it's the system clock prescaled by 8
#define F_TICK (F_CPU / 8)

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
volatile unsigned char tx_buf[TX_BUF_LEN];
volatile unsigned int tx_buf_head, tx_buf_tail;
volatile unsigned char nmea_ready;
volatile unsigned long last_pps_tick;
volatile unsigned long tenth_ticks;
volatile unsigned char gps_locked;
volatile unsigned char dst_mode;
volatile unsigned char ampm;
volatile unsigned char menu_pos;
volatile char tz_hour;
volatile unsigned int timer_hibits;
#ifdef TENTH_DIGIT
volatile unsigned char tenth_enable;
volatile unsigned char disp_tenth;
volatile unsigned char tenth_dp;
#endif
#ifdef COLONS
unsigned char colon_state;
#endif
unsigned long debounce_time;
unsigned char button_down;
unsigned char brightness;
unsigned int fw_version_year, utc_ref_year;
unsigned char fw_version_mon, utc_ref_mon;
unsigned char fw_version_day, utc_ref_day;

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
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifndef V3
		// We have to make !SS an output for the duration of SPI master mode.
		// If it's an input and becomes low, the SPI controller get screwed up.
		// Trouble is, it's connected to another device (the GPS module) that's
		// actively sending a signal. So to avoid making a short circuit, we
		// copy the current input state to the output port and just hope that
		// the signal doesn't change for the duration of the SPI transaction.
		if (PINA & _BV(PINA7)) {
			PORTA |= _BV(PORTA7);
		} else {
			PORTA &= ~_BV(PORTA7);
		}
		DDRA |= _BV(DDA7); // Make !SS an output
		SPCR = _BV(SPE) | _BV(MSTR); // And turn on SPI
#endif

		// Now assert !CS
		PORT_MAX &= ~BIT_MAX_CS;

		SPDR = addr;
		while(!(SPSR & _BV(SPIF))) ;

		SPDR = val;
		while(!(SPSR & _BV(SPIF))) ;

		// And finally, release !CS.
		PORT_MAX |= BIT_MAX_CS;

#ifndef V3
		SPCR = 0; // Turn off SPI
		DDRA &= ~_BV(DDA7); // and revert the !SS pin back to an input.
		//PORTA &= ~_BV(PORTA7); // This doesn't really matter. Pull-ups are via PUEx.
#endif
	}
}

const unsigned char month_tweak[] PROGMEM = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

static inline unsigned char first_sunday(unsigned char m, unsigned int y) {
	// first, what's the day-of-week for the first day of whatever month?
	// From http://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week
	y -= m < 3;
	unsigned char month_tweak_val = pgm_read_byte(&(month_tweak[m - 1]));
	unsigned char dow = (y + y/4 - y/100 + y/400 + month_tweak_val + 1) % 7;

	// If the 1st is a Sunday, then the answer is 1. Otherwise, we count
	// up until we find a Sunday.
	return (dow == 0)?1:(8 - dow);
}

static inline unsigned char calculateDSTAU(const unsigned char d, const unsigned char m, const unsigned int y) {
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
static inline unsigned char calculateDSTNZ(const unsigned char d, const unsigned char m, const unsigned int y) {
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
                        while(change_day + 7 <= 30) change_day += 7; // last Sunday
                        if (d < change_day) return DST_NO;
                        else if (d == change_day) return DST_BEGINS;
                        else return DST_YES;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static inline unsigned char calculateDSTEU(const unsigned char d, const unsigned char m, const unsigned int y) {
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
                        while(change_day + 7 <= 31) change_day += 7; // last Sunday
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
                        while(change_day + 7 <= 31) change_day += 7; // last Sunday
                        if (d < change_day) return DST_YES;
                        else if (d == change_day) return DST_ENDS;
                        else return DST_NO;
                        break;
                default: // This is impossible, since m can only be between 1 and 12.
                        return 255;
        }
}
static inline unsigned char calculateDSTUS(const unsigned char d, const unsigned char m, const unsigned int y) {
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
static inline unsigned char calculateDST(const unsigned char d, const unsigned char m, const unsigned int y) {
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

static inline void startLeapCheck();

static inline void handle_time(char h, unsigned char m, unsigned char s, unsigned char dst_flags) {
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
		// For Europe, decisions are at 0100. Everywhere else it's 0200.
		unsigned char decision_hour = (dst_mode == DST_EU)?1:2;
		switch(dst_flags) {
			case DST_NO: dst_offset = 0; break; // do nothing
			case DST_YES: dst_offset = 1; break; // add one hour
			case DST_BEGINS:
				dst_offset = (h >= decision_hour)?1:0; // offset becomes 1 at 0200 (0100 EU)
				break;
                        case DST_ENDS:
				// The *summer time* hour has to be the decision hour,
				// and we haven't yet made 'h' the summer time hour,
				// so compare it to one less than the decision hour.
                                dst_offset = (h >= (decision_hour - 1))?0:1; // offset becomes 0 at 0200 (daylight) (0100 EU)
				break;
		}
		h += dst_offset;
		if (h >= 24) h -= 24;
	}

	// Every hour, check to see if the leap second value in the receiver is out-of-date
	unsigned char doLeapCheck = (m == 30 && s == 0);

	unsigned char am = 0;
	if (ampm) {
		// Create AM or PM
                if (h == 0) { h = 12; am = 1; }
		else if (h < 12) { am = 1; }
		else if (h > 12) h -= 12;
	}

	disp_buf[DIGIT_1_SEC] = (s % 10);
	disp_buf[DIGIT_10_SEC] = s / 10;
	disp_buf[DIGIT_1_MIN] = (m % 10);
	disp_buf[DIGIT_10_MIN] = m / 10;
	disp_buf[DIGIT_1_HR] = (h % 10);
	disp_buf[DIGIT_10_HR] = h / 10;
	disp_buf[DIGIT_100_MSEC] = disp_buf[DIGIT_MISC] = 0;
#ifndef COLONS
	// If we don't have colons, add decimal points as separators
	disp_buf[DIGIT_1_HR] |= MASK_DP;
	disp_buf[DIGIT_1_MIN] |= MASK_DP;
#endif
	if (ampm) {
#ifdef NEW_AMPM
		disp_buf[DIGIT_MISC] |= am ? MASK_AM : MASK_PM;
#else
// Early hardware used the digit 6 and 7 DPs for AM/PM.
		if (am)
			disp_buf[DIGIT_MISC] |= MASK_DP;
		else
			disp_buf[DIGIT_100_MSEC] |= MASK_DP;
#endif
	}
#ifdef COLONS
	if (colon_state == COLON_ON || ((colon_state == COLON_BLINK) && (s % 2 == 0))) {
		disp_buf[DIGIT_MISC] |= MASK_COLON_HM | MASK_COLON_MS;
	}
#endif

	if (doLeapCheck) startLeapCheck();
}

static inline void tx_char(const unsigned char c);
static inline void write_msg(const unsigned char *msg, const size_t length) {
	for(int i = 0; i < length; i++) {
		tx_char(msg[i]);
	}
 }
 
const unsigned char PROGMEM version_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x02, 0x01, 0x03, 0x0d, 0x0a };
static inline void startVersionCheck(void) {
	// Ask for the firnware version. We expect a 0x80 in response
	unsigned char msg[sizeof(version_msg)];
	memcpy_P(msg, version_msg, sizeof(version_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM leap_check_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x64, 0x20, 0x44, 0x0d, 0x0a };
static inline void startLeapCheck(void) {
	// Ask for the time message. We expect a 0x64-0x8e in response
	unsigned char msg[sizeof(leap_check_msg)];
	memcpy_P(msg, leap_check_msg, sizeof(leap_check_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM leap_update_msg[] = { 0xa0, 0xa1, 0x00, 0x04, 0x64, 0x1f, 0x00, 0x01, 0x7a, 0x0d, 0x0a };
static inline void updateLeapDefault(const unsigned char leap_offset) {
	// This is a set leap-second default message. It will write the given
	// offset to flash.
	unsigned char msg[sizeof(leap_update_msg)];
	memcpy_P(msg, leap_update_msg, sizeof(leap_update_msg));
	msg[6] = leap_offset;
	msg[8] ^= leap_offset; // fix the checksum
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM get_utc_ref_msg[] = { 0xa0, 0xa1, 0x00, 0x02, 0x64, 0x16, 0x72, 0x0d, 0x0a };
static inline void startUTCReferenceFetch() {
	// This is a request for UTC reference date message. We expect a 0x64-0x8a in response
	unsigned char msg[sizeof(get_utc_ref_msg)];
	memcpy_P(msg, get_utc_ref_msg, sizeof(get_utc_ref_msg));
	write_msg(msg, sizeof(msg));
}

const unsigned char PROGMEM utc_ref_msg[] = { 0xa0, 0xa1, 0x00, 0x08, 0x64, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x71, 0x0d, 0x0a };
static inline void updateUTCReference(const unsigned int y, const unsigned char mon, const unsigned char d) {
	// This sets the UTC reference date, which controls the boundaries of the GPS week window
	unsigned char msg[sizeof(utc_ref_msg)];
	memcpy_P(msg, utc_ref_msg, sizeof(utc_ref_msg));
	msg[7] = (unsigned char)(y >> 8);
	msg[8] = (unsigned char)y;
	msg[9] = mon;
	msg[10] = d;
	for(int i = 7; i <= 10; i++) msg[12] ^= msg[i]; // fix checksum
	write_msg(msg, sizeof(msg));
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

static inline void handleGPS(unsigned char binaryOnly) {
	unsigned int str_len = rx_str_len; // rx_str_len is where the \0 was written.

	if (str_len >= 3 && rx_buf[0] == 0xa0 && rx_buf[1] == 0xa1) { // binary protocol message
		unsigned int payloadLength = (((unsigned int)rx_buf[2]) << 8) | rx_buf[3];
		if (str_len != payloadLength + 7) return; // the A0, A1 bytes, length and checksum are added
		unsigned int checksum = 0;
		for(int i = 0; i < payloadLength; i++) checksum ^= rx_buf[i + 4];
		if (checksum != rx_buf[payloadLength + 4]) return; // checksum mismatch
		if (rx_buf[4] == 0x80) {
			fw_version_year = rx_buf[12 + 3];
			fw_version_mon = rx_buf[13 + 3];
			fw_version_day = rx_buf[14 + 3];
		} else if (rx_buf[4] == 0x64 && rx_buf[5] == 0x8a) {
			utc_ref_year = (rx_buf[3 + 4] << 8) | rx_buf[3 + 5];
			utc_ref_mon = rx_buf[3 + 6];
			utc_ref_day = rx_buf[3 + 7];
		} else if (rx_buf[4] == 0x64 && rx_buf[5] == 0x8e) {
			if (!(rx_buf[15 + 3] & (1 << 2))) return; // GPS leap seconds invalid
			if (rx_buf[13 + 3] == rx_buf[14 + 3]) return; // Current and default agree
			updateLeapDefault(rx_buf[14 + 3]);
		} else {
			return; // unknown binary protocol message
		}
	}

	if (binaryOnly) return; // we're not handling text sentences. 

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
		ptr = skip_commas(ptr, 1);
		if (ptr == NULL) return; // not enough commas
		gps_locked = *ptr == 'A'; // A = AOK
		ptr = skip_commas(ptr, 7);
		if (ptr == NULL) return; // not enough commas
		unsigned char d = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char mon = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned int y = (ptr[4] - '0') * 10 + (ptr[5] - '0');

		// We must turn the two digit year into the actual A.D. year number.
		// As time goes forward, we can keep a record of how far time has gotten,
		// and assume that time will always go forwards. If we see a date ostensibly
		// in the past, then it "must" mean that we've actually wrapped and need to
		// add 100 years. We keep this "reference" date in sync with the GPS receiver,
		// as it uses the reference date to control the GPS week rollover window.
  		y += 2000;
		while (y < utc_ref_year) y += 100; // If it's in the "past," assume time wrapped on us.
  
		if (utc_ref_year != 0 && y != utc_ref_year) {
			// Once a year, we should update the refence date in the receiver. If we're running on New Years,
			// then that's probably when it will happen, but anytime is really ok. We just don't want to do
			// it a lot for fear of burning the flash out in the GPS receiver.
  			updateUTCReference(y, mon, d);
			utc_ref_year = y;
			utc_ref_mon = mon;
			utc_ref_day = d;
		}

		// The problem is that our D/M/Y is UTC, but DST decisions are made in the local
		// timezone. We can adjust the day against standard time midnight, and
		// that will be good enough. Don't worry that this can result in d being either 0
		// or past the last day of the month. Those will still be more or less than the "decision day"
		// for DST, which is all that really matters.
		if (h + tz_hour < 0) d--;
		if (h + tz_hour > 23) d++;
		unsigned char dst_flags = calculateDST(d, mon, y);
		handle_time(h, min, s, dst_flags);
	}
}

ISR(USART0_RX_vect) {
	unsigned char rx_char = UDR0;
 
	if (nmea_ready) return; // ignore serial until current buffer handled 
	if (rx_str_len == 0 && !(rx_char == '$' || rx_char == 0xa0)) return; // wait for a "$" or A0 to start the line.

	rx_buf[rx_str_len] = rx_char;

	if (++rx_str_len == RX_BUF_LEN) {
		// The string is too long. Start over.
		rx_str_len = 0;
	}

	// If it's an ASCII message, then it's ended with a CRLF.
	// If it's a binary message, then it's ended when it's the correct length
	if ( (rx_buf[0] == '$' && (rx_char == 0x0d || rx_char == 0x0a)) ||
		(rx_buf[0] == 0xa0 && rx_str_len >= 4 && rx_str_len >= ((rx_buf[2] << 8) + rx_buf[3] + 7)) ) {
		rx_buf[rx_str_len] = 0; // null terminate
		nmea_ready = 1; // Mark it as ready
		return;
	}

}

ISR(USART0_UDRE_vect) {
	if (tx_buf_head == tx_buf_tail) {
		// the transmit queue is empty.
		UCSR0B &= ~_BV(UDRIE0); // disable the TX interrupt
		return;
	}
	UDR0 = tx_buf[tx_buf_tail];
	if (++tx_buf_tail == TX_BUF_LEN) tx_buf_tail = 0; // point to the next char
}

// If the TX buffer fills up, then this method will block, which should be avoided.
static inline void tx_char(const unsigned char c) {
	int buf_in_use;
	do {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			buf_in_use = tx_buf_head - tx_buf_tail;
		}
		if (buf_in_use < 0) buf_in_use += TX_BUF_LEN;
		wdt_reset(); // we might be waiting a while.
	} while (buf_in_use >= TX_BUF_LEN - 2) ; // wait for room in the transmit buffer

	tx_buf[tx_buf_head] = c;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// this needs to be atomic, because an intermediate state is tx_buf_head
		// pointing *beyond* the end of the buffer.
		if (++tx_buf_head == TX_BUF_LEN) tx_buf_head = 0; // point to the next free spot in the tx buffer
	}
	UCSR0B |= _BV(UDRIE0); // enable the TX interrupt. If it was disabled, then it will trigger one now.
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

static inline unsigned long timer_value() __attribute__ ((always_inline));
static inline unsigned long timer_value() {
	unsigned long now;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		// We have to be careful. It's possible that we capture TCNTn at the same
		// time as an overflow. If that happens, it means that the hibits value
		// is 1 too low. We can detect this by checking if the low bits are
		// "close" to zero and if the overflow interrupt is pending. If that's
		// true, then we can compensate locally for the missing interrupt (and
		// it will happen when we return anyway). If the low bits are high,
		// then the interrupt is pending because it came (shortly) after we sampled, so
		// we don't compensate. "close" can simply be testing the MSB for 0.
#ifdef V3
		now = (((unsigned long)timer_hibits) << 16) | TCNT2;
		if ((TIFR2 & _BV(TOV2)) && !(now & 0x8000)) now += 0x10000L;
#else
		now = (((unsigned long)timer_hibits) << 16) | TCNT1;
		if ((TIFR1 & _BV(TOV1)) && !(now & 0x8000)) now += 0x10000L;
#endif
	}
	return now;
}

#ifdef V3
ISR(TIMER2_OVF_vect) {
#else
ISR(TIMER1_OVF_vect) {
#endif
	timer_hibits++;
}

#ifdef V2
#ifdef V3
ISR(TIMER2_CAPT_vect) {
	unsigned long this_tick = (((unsigned long)timer_hibits) << 16) | ICR2;
#else
ISR(TIMER1_CAPT_vect) {
	unsigned long this_tick = (((unsigned long)timer_hibits) << 16) | ICR1;
#endif
#else
ISR(INT0_vect) {
	unsigned long this_tick = (((unsigned long)timer_hibits) << 16) | TCNT1;
#endif

#ifdef V3
	if ((TIFR2 & _BV(TOV2)) && !(this_tick & 0x8000)) this_tick += 0x10000L;
#else
	if ((TIFR1 & _BV(TOV1)) && !(this_tick & 0x8000)) this_tick += 0x10000L;
#endif

#ifdef TENTH_DIGIT
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
#endif
	last_pps_tick = this_tick;
	if (last_pps_tick == 0) last_pps_tick++; // it can never be zero

	if (menu_pos) return;
	if (!gps_locked) {
		write_no_sig();
		return;
	}

        unsigned char decode_mask = (unsigned char)~_BV(DIGIT_MISC); // assume decoding for all digits
	// If we are doing 12 hour display and if the 10 hours digit is 0, then blank it instead.
	// Its value will be zero, so simply disabling the hex decode will result in no segments.
	if (ampm && disp_buf[DIGIT_10_HR] == 0) {
		decode_mask &= ~_BV(DIGIT_10_HR); // No decode for tens-of-hours digit
	}
#ifdef TENTH_DIGIT
	// If we're not going to show the tenths...
	if (tenth_ticks == 0) {
		decode_mask &= ~_BV(DIGIT_100_MSEC); // No decode for tenth digit
	} else {
		disp_buf[DIGIT_1_SEC] |= MASK_DP; // add a decimal point on seconds digit
	}

	disp_tenth = 0; // right now, 0 is showing.

	// Watch out, there's an old bug lurking here. disp_buf[] gets
	// updated with data for the *next* second early on during *this*
	// second. If the tenth DP is ever used for anything time related,
	// (it used to be used for PM), then it will wind up changing *early*
	// if you're not careful.
	tenth_dp = (disp_buf[DIGIT_100_MSEC] & MASK_DP) != 0;
#else
	decode_mask &= ~_BV(DIGIT_100_MSEC); // No decode for tenth digit
#endif
	write_reg(MAX_REG_DEC_MODE, decode_mask);

	// Copy the display buffer data into the display, but do the least
	// significant digits first, for great justice.
	for(int i = sizeof(disp_buf) - 1; i >= 0; i--) {
		write_reg(MAX_REG_MASK_BOTH | i, disp_buf[i]);
	}
}

static unsigned char check_buttons() {
	unsigned long now = timer_value();
	if (debounce_time != 0 && now - debounce_time < DEBOUNCE_TICKS) {
		// We don't pay any attention to the buttons during debounce time.
		return 0;
	} else {
		debounce_time = 0; // debounce is over
	}
	unsigned char status = PORT_SW & (SW_0_BIT | SW_1_BIT);
	status ^= (SW_0_BIT | SW_1_BIT); // invert the buttons - 0 means down.
	if (!((button_down == 0) ^ (status == 0))) return 0; // either no button is down, or a button is still down

	// Something *changed*, which means we must now start a debounce interval.
	debounce_time = now;
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
#ifdef TENTH_DIGIT
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
#endif
#ifdef COLONS
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
				case COLON_BLINK: // blink - write to only P0. We don't actually blink the clock this way, though.
					write_reg(MAX_REG_MASK_P0 | 7, MASK_COLON_HM | MASK_COLON_MS);
					break;
			}
			break;
#endif
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
			// we're entering the menu system. Disable the tenth digit.
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
#ifdef TENTH_DIGIT
		case 4:
			eeprom_write_byte(EE_TENTHS, tenth_enable);
			break;
#endif
#ifdef COLONS
		case 5:
			eeprom_write_byte(EE_COLONS, colon_state);
			break;
#endif
		case 6:
			eeprom_write_byte(EE_BRIGHTNESS, brightness);
			break;
	}
	if (++menu_pos > 6) menu_pos = 0;
#ifndef TENTH_DIGIT
	// There is no tenth digit menu. Skip past it.
	if (menu_pos == 4) menu_pos++;
#endif
#ifndef COLONS
	// There is no colon menu. Skip past it.
	if (menu_pos == 5) menu_pos++;
#endif
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
#ifdef TENTH_DIGIT
		case 4: // tenths enabled
			tenth_enable = !tenth_enable;
			break;
#endif
#ifdef COLONS
		case 5: // colons
			if (++colon_state > COLON_STATE_MAX) colon_state = 0;
			break;
#endif
		case 6: // brightness
			brightness = ((brightness + 4) & 0xf) | 0x3;
			break;
	}
	menu_render();
}

// main() never returns.
void __ATTR_NORETURN__ main(void) {

	wdt_enable(WDTO_250MS);

	// Leave on only the parts of the chip we use.
#ifdef V3
	PRR = ~(_BV(PRSPI) | _BV(PRUSART0) | _BV(PRTIM2));
#else
	PRR = ~(_BV(PRSPI) | _BV(PRUSART0) | _BV(PRTIM1));
#endif

	// Make sure the CS pin is high and everything else is low.
	PORT_MAX = BIT_MAX_CS;
	DDRA = DDR_BITS_A;

	DDRB = DDR_BITS_B;
#ifdef V31
	PUEA = PULLUP_BITS_A;
#else
	PUEB = PULLUP_BITS_B;
#endif

	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A = _BV(U2X0);
#else
	UCSR0A = 0;
#endif

	UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0); // RX interrupt and TX+RX enable

	// 8N1
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

#ifndef V2
	MCUCR = _BV(ISC01) | _BV(ISC00); // INT0 on rising edge
	GIMSK = _BV(INT0); // enable INT0
#endif

	tx_buf_head = tx_buf_tail = 0;
	rx_str_len = 0;
	nmea_ready = 0;

#ifdef V2
#ifdef V3
	TCCR2B = _BV(ICES2) | _BV(CS21); // prescale by 8, capture on rising edge
	TIMSK2 = _BV(TOIE2) | _BV(ICIE2); // interrupt on capture or overflow
#else
	TCCR1B = _BV(ICES1) | _BV(CS11); // prescale by 8, capture on rising edge
	TIMSK1 = _BV(TOIE1) | _BV(ICIE1); // interrupt on capture or overflow
#endif
#else
	TCCR1B = _BV(CS11); // prescale by 8
	TIMSK1 = _BV(TOIE1); // interrupt on overflow
#endif

	timer_hibits = 0;

	// For V2, we can't do this here. We can only turn on SPI
	// while PA7 is an output.
#ifdef V3
	SPCR = _BV(SPE) | _BV(MSTR);
#endif
	// Go as fast as possible.
	SPSR = _BV(SPI2X);

	unsigned char ee_rd = eeprom_read_byte(EE_TIMEZONE);
	if (ee_rd == 0xff)
		tz_hour = -8;
	else
		tz_hour = ee_rd - 12;
	dst_mode = eeprom_read_byte(EE_DST_MODE);
	if (dst_mode > DST_MODE_MAX) dst_mode = DST_US;
	ampm = eeprom_read_byte(EE_AM_PM) != 0;

#ifdef TENTH_DIGIT
	tenth_enable = eeprom_read_byte(EE_TENTHS) != 0;
#endif
#ifdef COLONS
        colon_state = eeprom_read_byte(EE_COLONS);
	if (colon_state > COLON_STATE_MAX) colon_state = 1; // default to just on.
#endif

	gps_locked = 0;
	menu_pos = 0;
	debounce_time = 0;
	button_down = 0;
	last_pps_tick = 0;
#ifdef TENTH_DIGIT
	tenth_ticks = 0;
	disp_tenth = 0;
#endif

	// Turn off the shut-down register, clear the digit data
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E);
	write_reg(MAX_REG_SCAN_LIMIT, 7); // display all 8 digits
	write_reg(MAX_REG_DEC_MODE, 0);
	brightness = (eeprom_read_byte(EE_BRIGHTNESS) & 0xf) | 0x3;
	write_reg(MAX_REG_INTENSITY, brightness);

	// Turn on the self-test for a second
	write_reg(MAX_REG_TEST, 1);
	Delay(1000);
	write_reg(MAX_REG_TEST, 0);

	menu_pos = 99; // hack to disable PPS processing

	// Turn on interrupts
	sei();

	fw_version_year = fw_version_mon = fw_version_day = 0;
	startVersionCheck();
	unsigned long start = timer_value();
	do {
		wdt_reset();
		if (nmea_ready) {
			handleGPS(1);
			rx_str_len = 0; // clear the buffer
			nmea_ready = 0;
			if (fw_version_year != 0) {
				write_reg(MAX_REG_DEC_MODE, 0x3f); // first 6 digits
				write_reg(MAX_REG_MASK_BOTH | DIGIT_10_HR, fw_version_year / 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_1_HR, fw_version_year % 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_10_MIN, fw_version_mon / 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_1_MIN, fw_version_mon % 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_10_SEC, fw_version_day / 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_1_SEC, fw_version_day % 10);
				fw_version_year = 0; // don't come back in here.
				// we can't "stack" binary commands, so we need to do this after the first finished.
				startUTCReferenceFetch();
			}
		}
	} while(timer_value() - start < 2 * F_TICK); // 2 seconds
	menu_pos = 0; // end-of-hack

	write_no_sig();

	while(1) {
		wdt_reset();
		if (nmea_ready) {
			// Do this out here so it's not in an interrupt-disabled context.
			handleGPS(0);
			rx_str_len = 0; // now clear the buffer
			nmea_ready = 0;
			continue;
		}
		unsigned long local_lpt, now;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			local_lpt = last_pps_tick;
			now = timer_value();
		}
		// If we've not seen a PPS pulse in a certain amount of time, then
		// without doing something like this, the wrong time would just get stuck.
		if (local_lpt != 0 && now - local_lpt > LOST_PPS_TICKS) {
			write_no_sig();
			last_pps_tick = 0;
			continue;
		}
#ifdef TENTH_DIGIT
		if (tenth_ticks != 0) {
			unsigned long current_tick = now - local_lpt;
			unsigned char ldt;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
				ldt = disp_tenth;
			}
			unsigned int current_tenth = (unsigned int)((current_tick / tenth_ticks) % 10);
			// if ldt is 0 and current_tenth is 9, then that means that
			// the ISR changed disp_tenth out from under us. In that
			// case, we don't really want to write the 9 on top of the
			// zero that just happend. Next time we come through, though,
			// current_tenth will be 0 since last_pps_tick will have changed.
			if (ldt != current_tenth && !(ldt == 0 && current_tenth == 9)) {
				// This is really only volatite during the 0 tenth ISR.
				disp_tenth = current_tenth;
				// Write the tenth-of-a-second digit, preserving the
				// decimal point state (just in case)
				// We don't do this on tenth zero, though, becuase SPI
				// blocks interrupts and we don't want to delay the zero
				// second interrupt, which in principle should happen simultaneously.
				// We also don't need to write the tenth digit during tenth zero because
				// the PPS handler already did it for us.
				if (current_tenth != 0)
					write_reg(MAX_REG_MASK_BOTH | DIGIT_100_MSEC, current_tenth | (tenth_dp ? MASK_DP : 0));
			}
		}
#endif
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
