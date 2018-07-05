/*
 * Display.c
 *
 *  Created on: Jun 28, 2018
 *      Author: Zach Ash
 *      Alec Willison
 *      Modified code from Valvanoware
 */
/* Includes */
#include "Display.h"
#include "driverlib.h"
#include "Common.h"
#include <stdbool.h>

/* Signatures */
void Display_Init(void);
void Display_Backlight(bool turnOn);
char Display_SendCommand(char data);
char Display_SendData(char data);
void Display_DrawRect(int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t color);
void Display_DrawVerticalLine(int16_t x, int16_t y,
    int16_t h, uint16_t color);
void Display_SetRotation(uint8_t m);
void static Display_SetAddrWindow(uint8_t x0, uint8_t y0,
    uint8_t x1, uint8_t y1);
void static Display_CommandList(const uint8_t *addr);
void static Display_Color(uint16_t color);
void Display_Delay1ms(int n);

static uint8_t Rotation;
static int16_t _width = DISPLAY_TFTWIDTH;
static int16_t _height = DISPLAY_TFTHEIGHT;

/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig = {
        EUSCI_SPI_CLOCKSOURCE_SMCLK,                           // SMCLK Clock Source
        12000000,                                              // SMCLK Current SMCLK Freq
        4000000,                                               // SPICLK = 4MHz
        EUSCI_SPI_MSB_FIRST,                                   // MSB First
        EUSCI_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT, // Phase
        EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_LOW,                // low polarity
        EUSCI_SPI_3PIN                                         // 3Wire SPI Mode
};

/************************************************************
 * Display_Init: Initializes the display to run at 4MHz and
 * initializes all necessary ports and pins as well as SPI
 * on module B0.
 * P6.6 is used for controling the LED backlight
 * P3.6 is used for resetting the LED
 * P5.0 is used for chip select for the SPI module
 * P5.2 is used for the D/C for the SPI module
 * P1.5 is used for SPI clk
 * P1.6 is used for SPI MOSI
 ************************************************************/
void Display_Init(void)
{
    /* Backlight direction */
    P6DIR |= BIT6;
    Display_Backlight(true);

    // Toggle 3.6 - RESET
    P3DIR |= BIT6;
    P3OUT |= BIT6;
    Common_Delay_ms(500);
    P3OUT &= ~BIT6;
    Common_Delay_ms(500);
    P3OUT |= BIT6;

    //Config
    P5DIR |= BIT0 | BIT2;
    P5OUT |= ~(BIT0 | BIT2);
    P1SEL0 |= (BIT4 | BIT5 | BIT6 | BIT7);
    P1SEL1 &=~ (BIT4 | BIT5 | BIT6 | BIT7);

    /* Configuring SPI in 3wire master mode */
    MAP_SPI_initMaster(EUSCI_B0_SPI_BASE, &spiMasterConfig);
    /* Enable SPI module */
    MAP_SPI_enableModule(EUSCI_B0_SPI_BASE);

    //Commands for setting up the display. See display.h
    Display_CommandList(Rcmd1);
    Display_CommandList(Rcmd2);
    Display_CommandList(Rcmd3);
}

/************************************************************
 * Display_Backlight: Allows the display LED to be turned on
 * or off.
 ************************************************************/
void Display_Backlight(bool turnOn)
{
    if(turnOn)
        P6OUT |= BIT6;
    else
        P6OUT &=~ BIT6;
}

/************************************************************
 * Display_DrawRect: Draws a rectangle of the specified
 * color starting at the specified location with
 * the specified with and height. The function first sets
 * the area to be drawn, then by sending data to the display,
 * a pointer is incremented to go through each pixel and set
 * its display RGB color (16 bit). From Valvanoware.
 ************************************************************/
void Display_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    uint8_t hi = color >> 8, lo = color;

    if((x + w - 1) >= _width)  w = _width  - x;
    if((y + h - 1) >= _height) h = _height - y;
    Display_SetAddrWindow(x, y, x+w-1, y+h-1);

    for(y=h; y>0; y--) {
        for(x=w; x>0; x--) {
            Display_SendData(hi);
            Display_SendData(lo);
        }
    }
}

////////////// PRIVATE FUNCTIONS ///////////////////////
/************************************************************
 * Display_CommandList: Sets up and runs a number of
 * initialization commands for the display. These are required
 * or the display will not work. From Valvanoware.
 ************************************************************/
void static Display_CommandList(const uint8_t *addr) {

    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *(addr++);               // Number of commands to follow
    while(numCommands--) {                 // For each command...
        Display_SendCommand(*(addr++));             //   Read, issue command
        numArgs  = *(addr++);                //   Number of args to follow
        ms       = numArgs & DELAY;          //   If hibit set, delay follows args
        numArgs &= ~DELAY;                   //   Mask out delay bit
        while(numArgs--) {                   //   For each argument...
            Display_SendData(*(addr++));              //     Read, issue argument
        }

        if(ms) {
            ms = *(addr++);             // Read post-command delay time (ms)
            if(ms == 255) ms = 500;     // If 255, delay for 500 ms
            Common_Delay_ms(ms);
        }
    }
}

/************************************************************
 * Display_SendCommand: Sends a command to the display. Commands
 * tell the display what is shall do. The function waits for
 * TX buf to be empty before sending the byte of data and waiting
 * for a byte return of data from the MCU
 ************************************************************/
char Display_SendCommand(char data)
{
    while((UCB0IFG&0x0002)==0x0000){};    // wait until UCB0TXBUF empty
    P5OUT &=~ BIT2; //Lo
    UCB0TXBUF = data;                        // command out
    while((UCB0IFG&0x0001)==0x0000){};    // wait until UCB0RXBUF full
    return UCB0RXBUF;
}

/************************************************************
* Display_SendData: Sends data to the display. Data is what is
* written to display after command. The function waits for
* TX buf to be empty before sending the byte of data and waiting
* for a byte return of data from the MCU
************************************************************/
char Display_SendData(char data)
{
    while((UCB0IFG&0x0002)==0x0000){};    // wait until UCA3TXBUF empty
    P5OUT |= BIT2; //DC hi
    UCB0TXBUF = data;                        // command out
    while((UCB0IFG&0x0001)==0x0000){};    // wait until UCA3RXBUF full
    return UCB0RXBUF;
}

/************************************************************
* Display_SetAddrWindow: Sets the area in the display that will
* be drawn. This is convenient because the window will be
* created, then a pointer is written to,to set the pixel color.
* A series of data and commands are sent to the display
* in order to set the window.
************************************************************/
void static Display_SetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

  Display_SendCommand(0x2A); // Column addr set
  Display_SendData(0x00);
  Display_SendData(x0);     // XSTART
  Display_SendData(0x00);
  Display_SendData(x1);     // XEND

  Display_SendCommand(0x2B); // Row addr set
  Display_SendData(0x00);
  Display_SendData(y0);     // YSTART
  Display_SendData(0x00);
  Display_SendData(y1);     // YEND

  Display_SendCommand(0x2C); // write to RAM
}
