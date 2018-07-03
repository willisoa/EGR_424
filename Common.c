/*
 * Common.c
 *
 *  Created on: Jun 28, 2018
 *      Author: willi
 */
#include "Common.h"
#include "Driverlib.h"

void Common_CLK_48MHz(void)
{
    const int iMCLOCK_FREQ = 32000;
    const int iXTL_FREQ = 48000000;

    //External 48MHz crystal
    //Configuring pins for peripheral/crystal usage
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_CS_setExternalClockSourceFrequency(iMCLOCK_FREQ, iXTL_FREQ); // enables getMCLK,
    /* Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_CS_startHFXT(false); // false means that there are no timeouts set,
    //will return when stable
    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
}

void Common_Delay_ms(const int ms)
{
    int i = 0;
    for(i = 0; i < ms; i++)
    {
        __delay_cycles(48000);
    }
}

