/*
 * ADC.c
 *
 *  Created on: Jun 21, 2018
 *      Author: willi
 */
#include "ADC.h"
#include "driverlib.h"

void ADC_ISR(void);

float ADC_Voltage;

void ADC_Init(void)
{
    /* For quicker floating point calculations */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

    /* Setting up GPIO pins as analog inputs. Since P5.5 is the ADC pin, set in in tertiary mode to receive ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Init ADC to sample with SMCLK at 3MHz / 32 / 5 = 18750 Hz. We need < 16kHz for the project*/
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_32, ADC_DIVIDER_5, ADC_NOROUTE);
    /* Set up single sample mode to continuously run and set up memory. Use default 3.3V reference and set in non-differential mode */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, ADC_NONDIFFERENTIAL_INPUTS);
    /* Enabling sample timer in auto iteration mode and interrupts*/
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    /* Register the ADC ISR, and enable INT0 (P5.5 conversion) and ADC module interrupts. Enable master */
    MAP_ADC14_registerInterrupt(ADC_ISR);
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    MAP_ADC14_enableModule();
    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
}

float ADC_GetVoltage(void)
{
    return ADC_Voltage;
}

void ADC_ISR(void)
{
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    /* Clear interrupt flag */
    MAP_ADC14_clearInterruptFlag(status);

    if (ADC_INT0 & status)
        /* Get the voltage from the ADC input */
        ADC_Voltage = MAP_ADC14_getResult(ADC_MEM0) * 3.3 / 16384; //16384 = 2^14
}



