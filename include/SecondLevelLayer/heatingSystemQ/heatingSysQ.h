/*
 * HeatingSystemQ.h
 *
 *  Created on: Jun 14, 2018
 *      Author: unix
 */

#ifndef HEATINGSYSQ_H_
#define HEATINGSYSQ_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "hardware_init.h"

void heatingSysQueueSend(void);
void initheatingSysQueue(void);
void heatingSysQueueRead(void);

/* declare global Queue instance*/
extern QueueHandle_t xQueueHeatingSys;
/* declare global instance of the struct*/

 /*define interfaces for the Matlab component (Always after the struct)*/
#define autoProgramSelected xMessageHeatingSys.autoProgramSelected
#define manualProgramSelected xMessageHeatingSys.manualProgramSelected
#define autoProgramTimeEnabled xMessageHeatingSys.autoProgramTimeEnabled
#define stateOfHeatingSystem xMessageHeatingSys.stateOfHeatingSystem
#define smartCntFlag xMessageHeatingSys.smartCntFlag

#define SOLLtemperature xMessageHeatingSys.SOLLtemperature
#define smartCntDown xMessageHeatingSys.smartCntDown
#define smartCntUp xMessageHeatingSys.smartCntUp
#define up xMessageHeatingSys.up
#define Temperature xMessageHeatingSys.Temperature

/*imported signals*/
extern GUIQ RxedMessage;
extern RtrEncdBtnQ RxedMessageRtr;

#define stateOfProgram RxedMessage.stateOfProgram
#define selectProgram RxedMessage.selectProgram

#define TurnDetected RxedMessageRtr.TurnDetected

#endif /* HEATINGSYSQ_H_ */
