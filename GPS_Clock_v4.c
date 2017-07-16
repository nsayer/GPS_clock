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

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>
#include <math.h>
#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

// 32 MHz
#define F_CPU (32000000UL)

// CLK2X = 0. For 9600 baud @ 32 MHz: 
#define BSEL (12)
#define BSCALE (4)

#include <util/delay.h>

// Port C is used for the display SPI interface and serial.
#define PORT_MAX PORTC
#define BIT_MAX_CS _BV(4)

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

#define PORT_SW PORTA.IN
#define SW_0_BIT _BV(1)
#define SW_1_BIT _BV(0)

// These are return values from the DST detector routine.
// DST is not in effect all day
#define DST_NO 0
// DST is in effect all day
#define DST_YES 1
// DST begins at 0200
#define DST_BEGINS 2
// DST ends 0200 - that is, at 0100 pre-correction.
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

// This is the timer frequency - it's the system clock prescaled by 1
// Keep this synced with the configuration of Timer C4!
#define F_TICK (F_CPU / 1)

// We want something like 50 ms. - 1/20 sec
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
volatile unsigned char nmea_ready;
volatile unsigned long last_pps_tick;
volatile unsigned char last_pps_tick_good;
volatile unsigned long tenth_ticks;
volatile unsigned char gps_locked;
volatile unsigned char ampm;
volatile unsigned char menu_pos;
volatile unsigned char tenth_enable;
volatile unsigned char disp_tenth;
volatile unsigned char tenth_dp;
unsigned char dst_mode;
char tz_hour;
unsigned char colon_state;
unsigned long debounce_time;
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
	// Since we actually perform SPI operations in some interrupts,
	// we can't allow them to be interrupted.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {

		// Now assert !CS
		PORT_MAX.OUTCLR = BIT_MAX_CS;

		SPIC.DATA = addr;
		while(!(SPIC.STATUS & SPI_IF_bm)) ;

		SPIC.DATA = val;
		while(!(SPIC.STATUS & SPI_IF_bm)) ;

		// And finally, release !CS.
		PORT_MAX.OUTSET = BIT_MAX_CS;

	}
}

static const unsigned char month_tweak[] PROGMEM = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

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
	// twice. We can't do other than that because we'd need to know that
	// the second after 59 is 60 instead of 0, and we can't know that.
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
	if (ampm) {
		disp_buf[DIGIT_MISC] |= am ? MASK_AM : MASK_PM;
	}
	if (colon_state == COLON_ON || ((colon_state == COLON_BLINK) && (s % 2 == 0))) {
		disp_buf[DIGIT_MISC] |= MASK_COLON_HM | MASK_COLON_MS;
	}

	if (doLeapCheck) startLeapCheck();
}

static inline void write_msg(const unsigned char *msg, const size_t length) {
	for(int i = 0; i < length; i++) {
		while(!(USARTC0.STATUS & USART_DREIF_bm)) ; // wait for ready
		USARTC0.DATA = msg[i];
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

static const char hexes[] PROGMEM = "0123456789abcdef";

static unsigned char hexChar(unsigned char c) {
	if (c >= 'A' && c <= 'F') c += ('a' - 'A'); // make lower case
	const char* outP = strchr_P(hexes, c);
	if (outP == NULL) return 0;
	return (unsigned char)(outP - hexes);
}

static inline void handleGPS() {
	unsigned int str_len = rx_str_len; // rx_str_len is where the \0 was written.

	if (str_len >= 3 && rx_buf[0] == 0xa0 && rx_buf[1] == 0xa1) { // binary protocol message
		unsigned int payloadLength = (((unsigned int)rx_buf[2]) << 8) | rx_buf[3];
		if (str_len != payloadLength + 5) return; // the A0, A1 bytes, length and checksum are added
		unsigned int checksum = 0;
		for(int i = 0; i < payloadLength; i++) checksum ^= rx_buf[i + 4];
		if (checksum != rx_buf[payloadLength + 4]) return; // checksum mismatch
		if (rx_buf[4] == 0x64 && rx_buf[5] == 0x8e) {
			if (!(rx_buf[15 + 3] & (1 << 2))) return; // GPS leap seconds invalid
			if (rx_buf[13 + 3] == rx_buf[14 + 3]) return; // Current and default agree
			updateLeapDefault(rx_buf[14 + 3]);
		} else {
			return; // unknown binary protocol message
		}
	}
 
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
		ptr = skip_commas(ptr, 8);
		if (ptr == NULL) return; // not enough commas
		unsigned char d = (ptr[0] - '0') * 10 + (ptr[1] - '0');
		unsigned char mon = (ptr[2] - '0') * 10 + (ptr[3] - '0');
		unsigned int y = (ptr[4] - '0') * 10 + (ptr[5] - '0');

		// Y2.1K bug here... We must turn the two digit year into
		// the actual A.D. year number. As time goes forward, in
		// principle, we could start deciding that "low" values
		// get 2100 added instead of 2000. You'd think that
		// way before then GPS will be obsolete, though.
		y += 2000;
		if (y < 2017) y += 100; // As I type this, it's A.D. 2017

		// Every year, on april fool's day at 30 seconds past midnight,
		// update the UTC reference date in the receiver.
		if (h == 0 && min == 0 && s == 30 && mon == 4 && d == 1) {
			updateUTCReference(y, mon, d);
		}

		// The problem is that our D/M/Y is UTC, but DST decisions are made in the local
		// timezone. We can adjust the day against standard time midnight, and
		// that will be good enough. Don't worry that this can result in d being either 0
		// or past the last day of the month. Neither of those will match the "decision day"
		// for DST, which is the only day on which the day of the month is significant.
		if (h + tz_hour < 0) d--;
		if (h + tz_hour > 23) d++;
		unsigned char dst_flags = calculateDST(d, mon, y);
		handle_time(h, min, s, dst_flags);
	} else if (!strncmp_P(ptr, PSTR("$GPGSA"), 6)) {
		// $GPGSA,A,3,02,06,12,24,25,29,,,,,,,1.61,1.33,0.90*01
		ptr = skip_commas(ptr, 2);
		if (ptr == NULL) return; // not enough commas
		gps_locked = (*ptr == '3' || *ptr == '2');
	}
}

ISR(USARTC0_RXC_vect) {
	unsigned char rx_char = USARTC0.DATA;
 
	if (nmea_ready) return; // ignore serial until current buffer is handled 
	if (rx_str_len == 0 && !(rx_char == '$' || rx_char == 0xa0)) return; // wait for a "$" or A0 to start the line.

	rx_buf[rx_str_len] = rx_char;
	if (rx_char == 0x0d || rx_char == 0x0a) {
		rx_buf[rx_str_len] = 0; // null terminate
		nmea_ready = 1;
		return;
	}
	if (++rx_str_len == RX_BUF_LEN) {
		// The string is too long. Start over.
		rx_str_len = 0;
	}
}

static void write_no_sig() {
	tenth_ticks = 0;
	last_pps_tick_good = 0;
	// Clear out the digit data
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E);
	write_reg(MAX_REG_DEC_MODE, 0);
        write_reg(MAX_REG_MASK_BOTH | 0, MASK_C | MASK_E | MASK_G); // n
        write_reg(MAX_REG_MASK_BOTH | 1, MASK_C | MASK_D | MASK_E | MASK_G); // o
        write_reg(MAX_REG_MASK_BOTH | 3, MASK_A | MASK_C | MASK_D | MASK_E | MASK_F | MASK_G); // G
        write_reg(MAX_REG_MASK_BOTH | 4, MASK_A | MASK_B | MASK_E | MASK_F | MASK_G); // P
        write_reg(MAX_REG_MASK_BOTH | 5, MASK_A | MASK_C | MASK_D | MASK_F | MASK_G); // S
}

// Note that this function MUST be called from an atomic block.
static inline unsigned long timer_value() __attribute__ ((always_inline));
static inline unsigned long timer_value() {
	// We've configured event block 0-3 for timer C 4/5 capture.
	// CCA causes an interrupt, but CCB doesn't, so use a
	// synthetic capture to grab the current value. This avoids
	// having to deal with overflow propagation issues.
	EVSYS.STROBE = _BV(1); // event channel 1
	while(!((TCC4.INTFLAGS & TC4_CCBIF_bm) && (TCC5.INTFLAGS & TC5_CCBIF_bm))) ; // wait for both words
	unsigned long out = (((unsigned long)TCC5.CCB) << 16) | TCC4.CCB;
	TCC4.INTFLAGS = TC4_CCBIF_bm; // XXX Why is this necessary?
	TCC5.INTFLAGS = TC5_CCBIF_bm;
	return out;
}

ISR(TCC5_CCA_vect) {
	unsigned long this_tick;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		while(!((TCC4.INTFLAGS & TC4_CCAIF_bm) && (TCC5.INTFLAGS & TC5_CCAIF_bm))) ; // wait for both words
		this_tick = (((unsigned long)TCC5.CCA) << 16) | TCC4.CCA;
		TCC4.INTFLAGS = TC4_CCAIF_bm; // XXX Why is this necessary?
		TCC5.INTFLAGS = TC5_CCAIF_bm;
	}
	if (last_pps_tick_good) {
		// DIY GPS driven FLL for the 32 MHz oscillator.
		unsigned long pps_tick_count = this_tick - last_pps_tick;
		if (pps_tick_count < F_CPU) DFLLRC32M.CALA++; // too slow
		else if (pps_tick_count > F_CPU) DFLLRC32M.CALA--; // too fast
	}

	// If we're in the menus, or if we've disabled 10ths or if this is
	// our first PPS (since good was set to 0), then we don't do the
	// tenth digit.
	if (menu_pos == 0 && tenth_enable && last_pps_tick_good) {
		tenth_ticks = (this_tick - last_pps_tick) / 10;
		// For unknown reasons we seemingly sometimes get spurious
		// PPS interrupts. If the math leads us to believe a
		// a tenth of a second is less than 50 ms worth of system clock,
		// then it's not right - just skip it.
		if (tenth_ticks < FAST_PPS_TICKS) tenth_ticks = 0;
	} else {
		tenth_ticks = 0;
	}
	last_pps_tick_good = 1;
	last_pps_tick = this_tick;

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

	write_reg(MAX_REG_DEC_MODE, decode_mask);
	// Copy the display buffer data into the display, but do the least
	// significant digits first, for great justice.
	for(int i = sizeof(disp_buf) - 1; i >= 0; i--) {
		write_reg(MAX_REG_MASK_BOTH | i, disp_buf[i]);
	}
}

static unsigned char check_buttons() {
	unsigned long now;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		now = timer_value();
	}
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
				case COLON_BLINK: // blink - write to only P0. We don't actually blink the clock this way, though.
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
			brightness = ((brightness + 4) & 0xf) | 0x3;
			break;
	}
	menu_render();
}

// main() never returns.
void __ATTR_NORETURN__ main(void) {

	// Run the CPU at 32 MHz.
	OSC.CTRL = OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)) ; // wait for it.

	_PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_RC32M_gc); // switch to it
	OSC.CTRL &= ~(OSC_RC2MEN_bm); // we're done with the 2 MHz osc.

	//wdt_enable(WDTO_1S); // This is broken on XMegas.
	// This replacement code doesn't disable interrupts (but they're not on now anyway)
	_PROTECTED_WRITE(WDT.CTRL, WDT_PER_256CLK_gc | WDT_ENABLE_bm | WDT_CEN_bm);
	while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take
	// We don't want a windowed watchdog.
	_PROTECTED_WRITE(WDT.WINCTRL, WDT_WCEN_bm);
	while(WDT.STATUS & WDT_SYNCBUSY_bm) ; // wait for it to take

	// Leave on only the parts of the chip we use.
	PR.PRGEN = PR_XCL_bm | PR_RTC_bm | PR_EDMA_bm;
	PR.PRPA = PR_DAC_bm | PR_ADC_bm | PR_AC_bm;
	PR.PRPC = PR_TWI_bm | PR_HIRES_bm;
	PR.PRPD = PR_USART0_bm | PR_TC5_bm;

	// Event 0 is PPS - it causes a timer capture.
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;
	EVSYS.CH0CTRL = 0;
	// Event 4 is a carry from timer 4 to timer 5
	EVSYS.CH4MUX = EVSYS_CHMUX_TCC4_OVF_gc;
	EVSYS.CH4CTRL = 0;

	PORTC.OUTSET = _BV(3) | _BV(4); // TXD and !D_CS default to high
	PORTC.DIRSET = _BV(3) | _BV(4) | _BV(5) | _BV(7);

	// Send an event on the rising edge of PPS.
	PORTC.PIN0CTRL = PORT_ISC_RISING_gc;

	// Switches get pull-ups.
	PORTA.PIN0CTRL = PORT_OPC_PULLUP_gc;
	PORTA.PIN1CTRL = PORT_OPC_PULLUP_gc;

	rx_str_len = 0;
	nmea_ready = 0;

	USARTC0.CTRLA = USART_DRIE_bm | USART_RXCINTLVL_LO_gc;
	USARTC0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
	USARTC0.CTRLD = 0;
	USARTC0.BAUDCTRLA = BSEL & 0xff;
	USARTC0.BAUDCTRLB = (BSEL >> 8) | (BSCALE << USART_BSCALE_gp);

	SPIC.CTRL = SPI_CLK2X_bm | SPI_ENABLE_bm | SPI_MASTER_bm; // As fast as possible, master mode.
	SPIC.INTCTRL = 0;
	SPIC.CTRLB = 0;

	TCC4.CTRLA = TC45_CLKSEL_DIV1_gc; // 32 MHz timer clocking - 31.25 ns granularity
	TCC4.CTRLB = 0;
	TCC4.CTRLC = 0;
	TCC4.CTRLD = TC45_EVSEL_CH0_gc; // capture on event A:0 B:1
	TCC4.CTRLE = TC45_CCBMODE_CAPT_gc | TC45_CCAMODE_CAPT_gc;
	TCC4.INTCTRLA = 0;
	TCC4.INTCTRLB = 0; // we'll use TCC5 for this

	TCC5.CTRLA = TC45_CLKSEL_EVCH4_gc; // Clock from timer 4's overflow
	TCC5.CTRLB = 0;
	TCC5.CTRLC = 0;
	TCC5.CTRLD = TC5_EVDLY_bm | TC45_EVSEL_CH0_gc; // We're cascading 32 bits - we must delay capture events 1 cycle
	TCC5.CTRLE = TC45_CCBMODE_CAPT_gc | TC45_CCAMODE_CAPT_gc;
	TCC5.INTCTRLA = 0;
	TCC5.INTCTRLB = TC45_CCAINTLVL_MED_gc;

	unsigned char ee_rd = eeprom_read_byte(EE_TIMEZONE);
	if (ee_rd == 0xff)
		tz_hour = -8;
	else
		tz_hour = ee_rd - 12;
	dst_mode = eeprom_read_byte(EE_DST_MODE);
	if (dst_mode > DST_MODE_MAX) dst_mode = DST_US;
	ampm = eeprom_read_byte(EE_AM_PM) != 0;

	tenth_enable = eeprom_read_byte(EE_TENTHS) != 0;
        colon_state = eeprom_read_byte(EE_COLONS);
	if (colon_state > COLON_STATE_MAX) colon_state = 1; // default to just on.

	gps_locked = 0;
	menu_pos = 0;
	debounce_time = 0;
	button_down = 0;
	last_pps_tick_good = 0;
	tenth_ticks = 0;
	disp_tenth = 0;

	// Enable high level of the interrupt controller
	PMIC.CTRL = PMIC_HILVLEN_bm;
	sei(); // turn interrupts on

	// Turn off the shut-down register, clear the digit data
	write_reg(MAX_REG_CONFIG, MAX_REG_CONFIG_R | MAX_REG_CONFIG_B | MAX_REG_CONFIG_S | MAX_REG_CONFIG_E);
	write_reg(MAX_REG_SCAN_LIMIT, 7); // display all 8 digits
	brightness = (eeprom_read_byte(EE_BRIGHTNESS) & 0xf) | 0x3;
	write_reg(MAX_REG_INTENSITY, brightness);

	// Turn on the self-test for a second
	write_reg(MAX_REG_TEST, 1);
	Delay(1000);
	write_reg(MAX_REG_TEST, 0);

	// Now enable the serial interrupt
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	startVersionCheck();
	unsigned long start = timer_value();
	do {
		wdt_reset();
		if (nmea_ready) {
			// This do block only exists so we can conveniently break out early.
			// It's an alternative to a goto.
			do {
				unsigned int str_len = rx_str_len; // rx_str_len is where the \0 was written.
				if (str_len < 5) break; // too short
				if (rx_buf[0] != 0xa0 || rx_buf[1] != 0xa1) break; // Not a binary msg
				unsigned int payloadLength = (((unsigned int)rx_buf[2]) << 8) | rx_buf[3];
				if (str_len != payloadLength + 5) break; // the A0, A1 bytes, length and checksum are added
				unsigned int checksum = 0;
				for(int i = 0; i < payloadLength; i++) checksum ^= rx_buf[i + 4];
				if (checksum != rx_buf[payloadLength + 4]) break; // checksum mismatch
				if (payloadLength < 14 || rx_buf[4] != 0x80) break; // wrong message
				write_reg(MAX_REG_DEC_MODE, 0x3f);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_10_HR, rx_buf[12 + 3] / 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_1_HR, rx_buf[12 + 3] % 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_10_MIN, rx_buf[13 + 3] / 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_1_MIN, rx_buf[13 + 3] % 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_10_SEC, rx_buf[14 + 3] / 10);
				write_reg(MAX_REG_MASK_BOTH | DIGIT_1_SEC, rx_buf[14 + 3] % 10);
			} while(0);
			rx_str_len = 0; // clear the buffer
			nmea_ready = 0;
		}
	} while(timer_value() - start < 2 * F_TICK);

	write_no_sig();

	// Now enable the pps interrupt
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	while(1) {
		wdt_reset();
		if (nmea_ready) {
			handleGPS();
			rx_str_len = 0; // now clear the buffer
			nmea_ready = 0;
			continue;
		}
		unsigned long local_lpt, local_tt;
		unsigned char local_dt, local_lptg;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			local_lpt = last_pps_tick;
			local_lptg = last_pps_tick_good;
			local_dt = disp_tenth;
			local_tt = tenth_ticks;
		}
		unsigned long now = timer_value();
		unsigned long current_tick = now - local_lpt;
		// If we've not seen a PPS pulse in a certain amount of time, then
		// without doing something like this, the wrong time would just get stuck.
		if (local_lptg && current_tick > LOST_PPS_TICKS) {
			write_no_sig();
			continue;
		}
		if (local_tt != 0) {
			unsigned int current_tenth = (unsigned int)((current_tick / local_tt) % 10);
			// if local_dt is 0 and current_tenth is 9, then that means that
			// the ISR changed disp_tenth out from under us. In that
			// case, we don't really want to write the 9 on top of the
			// zero that just happend. Next time we come through, though,
			// current_tenth will be 0 since last_pps_tick will have changed.
			if (local_dt != current_tenth && local_dt != 9) {
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
