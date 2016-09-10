/*
 * File:      hd44780.h
 * Date:      November 30, 2014
 * Author:    Craig Hollinger
 *
 * HD44780 display driver.  This code will work with common LCDs that use the
 * HD44780 controller.  Common displays have 2 rows with 16 characters each.
 * This code can work with displays with more rows and more characters, just
 * set the appropriate parameters when the hd44780_init() function is called.
 *
 * The HD44780 controller is used in 4-bit mode. The only restriction is that
 * the four bit data port be consecutive bits on one port only and either the
 * upper or lower four bits (nibble) of that port.
 *
 * The E and RS control lines can be on any other port and pin.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 3
 * or the GNU Lesser General Public License version 3, both as
 * published by the Free Software Foundation.
 */
#ifndef _HD44780_H_
#define _HD44780_H_ 1

/* This constant must be defined either here or globally in the project
   properties.  It tells the compiler what the main oscillator frequency
   is in hertz. */
// #define F_CPU (20000000UL)
/*
 * Initialize the LCD controller hardware.
 */
void hd44780_init(volatile uint8_t *dsplPt,
                  unsigned char dsplPn,
                  volatile uint8_t *ePt,
                  unsigned char ePn,
                  volatile uint8_t *rsPt,
                  unsigned char rsPn,
                  unsigned char row,
                  unsigned char col);

/*
 * Clear the LCD controller buffer.
 */
void hd44780_clear(void);

/*
 * Turn the display on or off.
 */
void hd44780_on(void);
void hd44780_off(void);
/*
 * Turn the cursor on or off.
 */
void hd44780_cursor_on(void);
void hd44780_cursor_off(void);
/*
 * Turn the cursor blinking on or off.
 */
void hd44780_cursor_blink(void);
void hd44780_cursor_no_blink(void);

/*
 * Clear the LCD line given by l.
 */
void hd44780_clearLine(unsigned char l);
/*
 * Send character c to the LCD display.
 */
void hd44780_putchar(unsigned char c);

/*
 * Send command c to the LCD display.
 */
void hd44780_putcmd(unsigned char c);

/*
 * Send a null terminated string stored in RAM to the LCD display.
 */
void hd44780_putstr(char str[]);

/*
 * Send the home command to the LCD display.
 */
void hd44780_home(void);

/*
 * Send a null terminated string stored in program memory to the LCD display.
 */
void hd44780_putstr_P(const char *addr);

/*
 * Move the cursor to the designated location on the screen.
 */
void hd44780_setCursor(unsigned char x, unsigned char y);

#endif /* _HD44780_H_ */
