/*
 * File:      hd44780.c
 * Date:      November 30, 2014
 * Author:    Craig Hollinger
 *
 * Description:
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
 * To get the _delay_ms() function to run accurately, the constant F_CPU must
 * be defined to the frequency of the main CPU oscillator in hertz.  For
 * example:
 *
 * #define F_CPU (16000000UL)
 *
 * or something similar.  It can be put in the hd44780.h file that goes with
 * this source file or defined globally in the IDE project properties.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 3
 * or the GNU Lesser General Public License version 3, both as
 * published by the Free Software Foundation.
 */

#include <stdint.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "lcd/hd44780.h"

/* The following are commands sent to the display to set it's various modes.
 */
 
#define LCD_CLEAR 0b00000001
#define LCD_HOME  0b00000010

/* Entry Mode set
 *
 * Sets display/cursor move direction, and whether the cursor moves or the
 * display shifts.
 */
#define LCD_ENTRY_SHFT_LEFT 0b00000100
#define LCD_ENTRY_SHFT_RGHT 0b00000110

#define LCD_ENTRY_SHFT_CURS 0b00000100
#define LCD_ENTRY_SHFT_DSPL 0b00000101

/* Display Control set
 *
 * Turns the display on or off, the cursor on or off, and cursor blink
 * on or off.
 */
#define LCD_DISPLAY_CNTL  0b00001000
#define LCD_DISPLAY_ON    0b00000100
#define LCD_CURSOR_ON     0b00000010
#define LCD_CURSOR_BLINK  0b00000001

/* Display Shift
 *
 * Sets cursor move or display shift,
 * and the direction (left/right).
 */
#define LCD_SHIFT_CURS 0b00010000
#define LCD_SHIFT_DSPL 0b00011000

#define LCD_SHIFT_LEFT 0b00010000
#define LCD_SHIFT_RGHT 0b00010100

/* Function Set
 *
 * Selects display interface length (4/8 bit),
 * number of display lines (1 or 2), and
 * character font (5x8/5x10).  If 2-line, cannot use 5x10 font (ignored).
 */
#define LCD_FNCN_4BIT  0b00100000
#define LCD_FNCN_8BIT  0b00110000

#define LCD_FNCN_1LINE 0b00100000
#define LCD_FNCN_2LINE 0b00101000

#define LCD_FNCN_FONT5X8   0b00100000
#define LCD_FNCN_FONT5X10  0b00100100

/*
 * Set the next character generator address to addr.
 */
#define LCD_CGADDR(addr) 	(0x40 | ((addr) & 0x3f))

/*
 * Set the next display address to addr.
 */
#define LCD_DDADDR(addr) 	(0x80 | ((addr) & 0x7f))

/*
 * Command to move cursor to start of line.
 */
#define LCD_LINE1 0x80
#define LCD_LINE2 0xc0
#define LCD_LINE3 0x94
#define LCD_LINE4 0xd4

/* Determines if the data written to LCD is a command or character
 */
 #define LCD_RS_CHARACTER 1
 #define LCD_RS_COMMAND 0
 
/* Local variables.
 */
uint8_t LcdE,        /* bit on port for the LCD E pin */
        LcdRS,       /* bit on port for the LCD RS pin */
        LcdPortMask, /* mask to preserve unused pins on LCD data port */
        rows,    /* maximum number of lines the display has */
        columns, /* maximum number of characters per line */
        displayControl;
volatile uint8_t *LcdEPort,    /* pointer to port for the LCD E pin */
                 *LcdRSPort,   /* pointer to port for the LCD RS pin */
                 *LcdDataPort; /* pointer to LCD data port */

void hd44780_setCursor(uint8_t x, uint8_t y);
void writeByte(uint8_t b, uint8_t rs);
void writeNibble(uint8_t n, uint8_t rs);

/*
 * hd44780_init()
 *
 * Set up pointers to the E and RS pins PORT registers, and the LCD port upper
 * or lower nibble.  Make all these pins output.
 *
 * Initialize the LCD controller.  The initialization sequence has a mandatory 
 * timing so the controller can safely recognize the type of interface desired.
 */
void hd44780_init(volatile uint8_t *dsplPt, /* port display is connected to */
                  uint8_t dsplPn,           /* pin that defines upper or lower nibble */
                  volatile uint8_t *ePt,    /* port that E pin is on */
                  uint8_t ePn,              /* pin number of E */
                  volatile uint8_t *rsPt,   /* port that RS pin is on */
                  uint8_t rsPn,             /* pin number of RS */
                  uint8_t row,              /* number of character rows */
                  uint8_t col)              /* number of characters in a row */
{
  rows = row;
  columns = col;

  /* save the pin numbers on the port */
  LcdE = ePn;
  LcdRS = rsPn;

  /* save the pointers to the ports */
  LcdEPort = ePt;
  LcdRSPort = rsPt;
  LcdDataPort = dsplPt;

  /* set up the HD44780 data port mask */
  if(dsplPn > 0x03)
  {
    LcdPortMask = 0xF0;// upper nibble
  }    
  else
  {
    LcdPortMask = 0x0F;/* lower nibble */
  }    

  /* make data port pins output */
  *(LcdDataPort - 1) |= LcdPortMask;

  /* set up pins for E and RS control lines */
  *(LcdEPort - 1) |= _BV(LcdE);
  *(LcdRSPort - 1) |= _BV(LcdRS);

  _delay_ms(15); /* Wait for the controller to wake up */

  writeNibble(LCD_FNCN_8BIT >> 4, LCD_RS_COMMAND);
  _delay_ms(5);

  writeNibble(LCD_FNCN_8BIT >> 4, LCD_RS_COMMAND);
  _delay_us(110);

  writeNibble(LCD_FNCN_8BIT >> 4, LCD_RS_COMMAND);
  _delay_us(110);

/* Now that we have the display's attention, set the 4-bit interface mode. */
  writeNibble(LCD_FNCN_4BIT >> 4, LCD_RS_COMMAND);
  _delay_us(100);

/* Now set up the display to two line mode. */
  writeByte(LCD_FNCN_4BIT|LCD_FNCN_2LINE|LCD_FNCN_FONT5X8, LCD_RS_COMMAND);
  _delay_us(100);

/* Set auto-increment and no display shift */
  writeByte(LCD_ENTRY_SHFT_RGHT|LCD_ENTRY_SHFT_CURS, LCD_RS_COMMAND);
  _delay_us(50);

/* Turn on the display and turn off the cursor. */
  displayControl = (LCD_DISPLAY_CNTL|LCD_DISPLAY_ON) & (~LCD_CURSOR_ON) & (~LCD_CURSOR_BLINK);
  writeByte(displayControl, LCD_RS_COMMAND);
  _delay_us(50);

/* Clear the display character buffer */
  writeByte(LCD_CLEAR, LCD_RS_COMMAND);
  _delay_ms(2);

}/* end hd74480_init() */

/*
 * pulseE()
 *
 * Send one pulse to the E signal (enable). Guarantee at least 500 ns of pulse
 * width.
 *
 * Make this function inline so that we don't incur the overhead of a function
 * call each time.
 */
static inline void pulseE(void) __attribute__((always_inline));
static inline void pulseE(void)
{
  *LcdEPort |= _BV(LcdE);

  _delay_us(1);

  *LcdEPort &= ~_BV(LcdE);

}/* end pulseE() */

/*
 * writeNibble()
 *
 * Send one nibble out to the LCD controller.  Expects the data to be in the 
 * lower nibble.
 */
void writeNibble(uint8_t n, uint8_t rs)
{
/* ensure the upper nibble is clear */
  n &= 0x0F;

/* assert RS as required */
  if (rs == 0)
    *LcdRSPort &= ~_BV(LcdRS);
  else
    *LcdRSPort |= _BV(LcdRS);

/* shift data into upper nibble if mask is upper nibble */
  if(LcdPortMask == 0xF0)
    n <<= 4;

/* read data port, OR in new data, then output */
  *LcdDataPort = (*LcdDataPort & ~LcdPortMask) | n;

  pulseE();

}/* end writeNibble() */

/*
 * writeByte()
 *
 * Send one byte to the LCD controller.  The controller is in 4-bit mode, send 
 * two nibbles.  If rs = 1, the byte is sent to the character generator.  If 
 * rs = 0, the byte is sent to the controller command register,
 */
void writeByte(uint8_t b, uint8_t rs)
{
  writeNibble(b >> 4, rs); /* send upper nibble first */
  writeNibble(b, rs);      /* lower nibble next */

}/* end writeByte() */

/*
 * hd44780_clear()
 *
 * Clear the display and set the cursor to the first character.
 */
void hd44780_clear(void)
{
  writeByte(LCD_CLEAR, LCD_RS_COMMAND);
  _delay_ms(2);

}/* end hd44780_clear() */

/*
 * hd44780_home()
 *
 * Send the home command to the LCD display (move the cursor to the first
 * character).
 */
void hd44780_home(void)
{
  writeByte(LCD_HOME, LCD_RS_COMMAND);
    _delay_ms(2);

}/* end hd44780_home() */

/*
 * hd44780_on()
 * hd44780_off()
 *
 * Turn the display on or off.
 */
void hd44780_on(void)
{
  displayControl |= LCD_DISPLAY_ON;
  writeByte(displayControl, LCD_RS_COMMAND);
  /* delay to allow the display to process the command */
  _delay_us(50);

}/* end hd44780_on() */

void hd44780_off(void)
{
  displayControl &= ~LCD_DISPLAY_ON;
  writeByte(displayControl, LCD_RS_COMMAND);
  /* delay to allow the display to process the command */
  _delay_us(50);

}/* end hd44780_off() */

/*
 * hd44780_cursor_on()
 * hd44780_cursor_off()
 *
 * Turn the cursor on or off.
 */
void hd44780_cursor_on(void)
{
  displayControl |= LCD_CURSOR_ON;
  writeByte(displayControl, LCD_RS_COMMAND);
  /* delay to allow the display to process the command */
  _delay_us(50);

}/* end hd44780_cursor_on() */

void hd44780_cursor_off(void)
{
  displayControl &= ~LCD_CURSOR_ON;
  writeByte(displayControl, LCD_RS_COMMAND);
  /* delay to allow the display to process the command */
  _delay_us(50);

}/* end hd44780_cursor_off() */

/*
 * hd44780_cursor_blink()
 * hd44780_cursor_no_blink
 *
 * Turn the cursor blinking on or off.
 */
void hd44780_cursor_blink(void)
{
  displayControl |= LCD_CURSOR_BLINK;
  writeByte(displayControl, LCD_RS_COMMAND);
  /* delay to allow the display to process the command */
  _delay_us(50);

}/* end hd44780_cursor_blink() */

void hd44780_cursor_no_blink(void)
{
  displayControl &= ~LCD_CURSOR_BLINK;
  writeByte(displayControl, LCD_RS_COMMAND);
  /* delay to allow the display to process the command */
  _delay_us(50);

}/* end hd44780_cursor_no_blink() */

/*
 * hd44780_clearLine()
 *
 * Clear the LCD line given by l.  The cursor is returned to the start of the
 * line.
 */
void hd44780_clearLine(uint8_t l)
{
  unsigned char i;

  if(l >= rows)
  {
    l = rows - 1;
  }

  hd44780_setCursor(0, l);

  for(i = 0; i < columns; i++)
  {
    writeByte((uint8_t)' ', LCD_RS_CHARACTER);
    _delay_us(50);
  }

  hd44780_setCursor(0, l);

}/* end hd44780_clearLine() */

/*
 * hd44780_purchar()
 *
 * Send character c to the LCD display.
 */
void hd44780_putchar(uint8_t c)
{
  writeByte(c, LCD_RS_CHARACTER);
  _delay_us(50);

}/* end hd44780_purchar() */

/*
 * hd44780_putcmd()
 *
 * Send command c to the LCD display.
 */
void hd44780_putcmd(uint8_t c)
{
  writeByte(c, LCD_RS_COMMAND);
  _delay_us(50);

}/* end hd44780_putcmd() */

/*
 * hd44780_setCursor()
 *
 *Send addresses x and y to the LCD display.
 */
void hd44780_setCursor(uint8_t x, uint8_t y)
{
  if(x >= columns)
  {
    x = columns - 1;
  }

  if(y >= rows)
  {
    y = rows - 1;
  }

  switch(y)
  {
    case 0:
      hd44780_putcmd(LCD_LINE1 + x);
    break;
    case 1:
      hd44780_putcmd(LCD_LINE2 + x);
    break;
    case 2:
      hd44780_putcmd(LCD_LINE3 + x);
    break;
    case 3:
      hd44780_putcmd(LCD_LINE4 + x);
    break;
    default:
    break;
  }/* end switch(y) */

}/* end hd44780_setCursor() */

/*
 * hd44780_putstr()
 *
 * Send a null terminated string stored in RAM to the LCD display.
 */
void hd44780_putstr(char str[])
{
  char c;
  int i = 0;

  while ((c = str[i++]) != 0)
  {
    hd44780_putchar((uint8_t)c);
  }
}/* end hd44780_putstr() */

/*
 * hd44780_putstr_P()
 *
 * Send a null terminated string stored in program memory to the LCD display.
 */
void hd44780_putstr_P(const char *addr)
{
  char c;

  while ((c = pgm_read_byte(addr++)) != 0)
  {
    hd44780_putchar((uint8_t)c);
  }
}/* end hd44780_putstr_P() */

