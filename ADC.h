/*
 * ADC.h
 *
 *  Created on: Jun 21, 2018
 *      Author: willi
 */

#ifndef ADC_H_
#define ADC_H_

void ADC_Init(void);
int ADC_ResultReady();
void ADC_ResultRead();
int ADC_GetResult();

#endif /* ADC_H_ */
