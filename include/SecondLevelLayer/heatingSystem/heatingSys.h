/*
 * heatingsys.h
 *
 *  Created on: Nov 27, 2015
 *      Author: unix
 */

#ifndef HEATINGSYS_H_
#define HEATINGSYS_H_

#include "hardware_init.h"
//#include "ds1307.h"
void updateSollTemperature(void);
void autoProgram();
void LEDfunction(void);
void initLEDs(void);
uint16_t returnDebugInfo();
void setSOLLTemperature(int16_t grad);

#endif /* HEATINGSYS_H_ */
