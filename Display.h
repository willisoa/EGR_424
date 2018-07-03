/*
 * Display.h
 *
 *  Created on: Jun 28, 2018
 *      Author: willi
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>
#include <stdbool.h>

#define DISPLAY_BLACK   0x0000
#define DISPLAY_BLUE    0xF800
#define DISPLAY_RED     0x001F
#define DISPLAY_GREEN   0x07E0
#define DISPLAY_CYAN    0xFFE0
#define DISPLAY_MAGENTA 0xF81F
#define DISPLAY_YELLOW  0x07FF
#define DISPLAY_WHITE   0xFFFF

#define DISPLAY_NOP     0x00
#define DISPLAY_SWRESET 0x01
#define DISPLAY_RDDID   0x04
#define DISPLAY_RDDST   0x09

#define DISPLAY_SLPIN   0x10
#define DISPLAY_SLPOUT  0x11
#define DISPLAY_PTLON   0x12
#define DISPLAY_NORON   0x13

#define DISPLAY_INVOFF  0x20
#define DISPLAY_INVON   0x21
#define DISPLAY_DISPOFF 0x28
#define DISPLAY_DISPON  0x29
#define DISPLAY_CASET   0x2A
#define DISPLAY_RASET   0x2B
#define DISPLAY_RAMWR   0x2C
#define DISPLAY_RAMRD   0x2E

#define DISPLAY_PTLAR   0x30
#define DISPLAY_COLMOD  0x3A
#define DISPLAY_MADCTL  0x36

#define DISPLAY_FRMCTR1 0xB1
#define DISPLAY_FRMCTR2 0xB2
#define DISPLAY_FRMCTR3 0xB3
#define DISPLAY_INVCTR  0xB4
#define DISPLAY_DISSET5 0xB6

#define DISPLAY_PWCTR1  0xC0
#define DISPLAY_PWCTR2  0xC1
#define DISPLAY_PWCTR3  0xC2
#define DISPLAY_PWCTR4  0xC3
#define DISPLAY_PWCTR5  0xC4
#define DISPLAY_VMCTR1  0xC5

#define DISPLAY_RDID1   0xDA
#define DISPLAY_RDID2   0xDB
#define DISPLAY_RDID3   0xDC
#define DISPLAY_RDID4   0xDD

#define DISPLAY_PWCTR6  0xFC

#define DISPLAY_GMCTRP1 0xE0
#define DISPLAY_GMCTRN1 0xE1

#define DISPLAY_TFTWIDTH  128
#define DISPLAY_TFTHEIGHT 160

#define DISPLAY_NOP     0x00
#define DISPLAY_SWRESET 0x01
#define DISPLAY_RDDID   0x04
#define DISPLAY_RDDST   0x09

#define DISPLAY_SLPIN   0x10
#define DISPLAY_SLPOUT  0x11
#define DISPLAY_PTLON   0x12
#define DISPLAY_NORON   0x13

#define DISPLAY_INVOFF  0x20
#define DISPLAY_INVON   0x21
#define DISPLAY_DISPOFF 0x28
#define DISPLAY_DISPON  0x29
#define DISPLAY_CASET   0x2A
#define DISPLAY_RASET   0x2B
#define DISPLAY_RAMWR   0x2C
#define DISPLAY_RAMRD   0x2E

#define DISPLAY_PTLAR   0x30
#define DISPLAY_COLMOD  0x3A
#define DISPLAY_MADCTL  0x36

#define DISPLAY_FRMCTR1 0xB1
#define DISPLAY_FRMCTR2 0xB2
#define DISPLAY_FRMCTR3 0xB3
#define DISPLAY_INVCTR  0xB4
#define DISPLAY_DISSET5 0xB6

#define DISPLAY_PWCTR1  0xC0
#define DISPLAY_PWCTR2  0xC1
#define DISPLAY_PWCTR3  0xC2
#define DISPLAY_PWCTR4  0xC3
#define DISPLAY_PWCTR5  0xC4
#define DISPLAY_VMCTR1  0xC5

#define DISPLAY_RDID1   0xDA
#define DISPLAY_RDID2   0xDB
#define DISPLAY_RDID3   0xDC
#define DISPLAY_RDID4   0xDD

#define DISPLAY_PWCTR6  0xFC

#define DISPLAY_GMCTRP1 0xE0
#define DISPLAY_GMCTRN1 0xE1

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in ROM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t
  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    DISPLAY_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    DISPLAY_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    DISPLAY_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    DISPLAY_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    DISPLAY_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    DISPLAY_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    DISPLAY_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    DISPLAY_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    DISPLAY_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    DISPLAY_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    DISPLAY_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    DISPLAY_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    DISPLAY_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    DISPLAY_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    DISPLAY_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 };                 //     16-bit color
static const uint8_t
  Rcmd2[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    DISPLAY_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    DISPLAY_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F };           //     XEND = 159
static const uint8_t
  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    DISPLAY_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    DISPLAY_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    DISPLAY_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    DISPLAY_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };

void Display_Init(void);
void Display_Backlight(bool turnOn);
void Display_DrawPixel(int16_t x, int16_t y, uint16_t color);
void Display_DrawVerticalLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void Display_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void Display_SetRotation(uint8_t m);

#endif /* DISPLAY_H_ */
