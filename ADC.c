/*
 * ADC.c
 *
 *  Created on: Jun 21, 2018
 *      Author: Zach Ash
 *      Alec Willison
 */
#include "ADC.h"
#include "SoundVisualizer.h"
#include "driverlib.h"

void Timer_32_1_ISR(void);
void ADC_ISR(void);

//The state of an ADC result
static int ADC_Ready = 0;
//The result of the ADC read
static int ADC_result = 0;
/************************************************************
 * ADC_Init: Initializes all modules needs for the ADC driver
 * to work properly and accurately. The ADC driver initializes
 * the MCUs FPU module for faster floating point calculations.
 * It also enables timer 32 for accurate sampling rates
 * (in this case, we're sampling at just above 17kHz). The ADC
 * module is set up to sample on P5.4 (ADC 1). Memory space is
 * set up for the sample to be held in. The ADC is in non-
 * repetative mode as timer32 will trigger conversions. An
 * interrupt is created for the ADC, as it will let the program
 * know when a sample has finished. The ADC is in singular mode,
 * i.e. it only gathers input from one pin, not two.
 ************************************************************/
void ADC_Init(void)
{
    /* Setting up GPIO pins as analog inputs.
     * Since P5.4 is the ADC pin, set in in tertiary mode to receive ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4,
        GPIO_TERTIARY_MODULE_FUNCTION);

    //Timer for precise sampling. 48MHz
    //Periodic mode so that it resets on count being met
    MAP_Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1,
        TIMER32_16BIT, TIMER32_PERIODIC_MODE);
    //Create interrupt for timer32
    MAP_Timer32_registerInterrupt(TIMER32_1_INTERRUPT, Timer_32_1_ISR);
    MAP_Timer32_enableInterrupt(TIMER32_1_BASE);
    // 48000000 / 16000 = 3000 ticks. Sampling at 16kHz
    MAP_Timer32_setCount(TIMER32_1_BASE, 3000);


    /* Run as fast as possible */
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1,
        ADC_DIVIDER_1, ADC_NOROUTE);
    /* Set up single sample mode to continuously run and set up memory.
     * Use default 3.3V reference and set in non-differential mode */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM1, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A1, ADC_NONDIFFERENTIAL_INPUTS);
    /* Register the ADC ISR, and enable INT1 P5.4 conversion)
     *  and ADC module interrupts. Enable master */
    MAP_ADC14_registerInterrupt(ADC_ISR);
    MAP_ADC14_enableInterrupt(ADC_INT1);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    MAP_ADC14_enableModule();
    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
    MAP_Timer32_startTimer(TIMER32_1_BASE, false);
}

/************************************************************
 * ADC_ResultReady: Returns the status of the ADC and whether
 * it has a sample ready or not.
 ************************************************************/
int ADC_ResultReady()
{
    return ADC_Ready;
}

/************************************************************
 * ADC_ResultRead: Sets the ADC result ready global variable
 * to not ready once a read is made from the adc.
 ************************************************************/
void ADC_ResultRead()
{
    ADC_Ready = 0;
}

/************************************************************
 * ADC_GetResult: Returns the result from the ADC result.
 ************************************************************/
int ADC_GetResult()
{
    return ADC_result;
}

/************************************************************
 * Timer_32_1_ISR: An interrupt service routine is created for
 * timer32 when the "tick" value is met. This ISR clears the
 * interrupt flag an starts an ADC conversion.
 ************************************************************/
void Timer_32_1_ISR(void)
{
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE);
    MAP_ADC14_toggleConversionTrigger();
}

/************************************************************
 * ADC_ISR: An interrupt service routine is created for
 * ADC14 on toggle conversion completed. The interrupt flag
 * is cleared and if ADC1 is the cause, then the
 * SoundVisualizer driver is called to store the reading.
 ************************************************************/
void ADC_ISR(void)
{
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    /* Clear interrupt flag */
    MAP_ADC14_clearInterruptFlag(status);

    if (ADC_INT1 & status)
    {
        /* Get the reading from the ADC input */
        ADC_result = MAP_ADC14_getResult(ADC_MEM1);
        ADC_Ready = 1;
        //save_reading(MAP_ADC14_getResult(ADC_MEM1));
    }
}



