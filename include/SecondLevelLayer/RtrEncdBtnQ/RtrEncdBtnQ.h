/*
 * RtrEncdBtnQ.h
 *
 *  Created on: Jun 14, 2018
 *      Author: unix
 */

#ifndef RTRENCDBTNQ_H_
#define RTRENCDBTNQ_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "hardware_init.h"

void rtrEncdBtnQueueSend(void);
void initRtrEncdBtnQueue(void);
void rtrEncdBtnQueueRead(void);

/* declare global Queue instance*/
extern QueueHandle_t xQueueRtrEncdBtn;

/*define the interface to target the struct*/
#define clicked xMessageRtrEncdBtn.clicked
#define doubleClicked xMessageRtrEncdBtn.doubleClicked
#define onStateofProgram xMessageRtrEncdBtn.onStateofProgram
#define holdCnt xMessageRtrEncdBtn.holdCnt
//step between Heating/Cooling
#define smartCntUp xMessageRtrEncdBtn.smartCntUp
#define smartCntDown xMessageRtrEncdBtn.smartCntDown
#define smartCntFlag xMessageRtrEncdBtn.smartCntFlag
#define signalButton xMessageRtrEncdBtn.signalButton
#define up xMessageRtrEncdBtn.up
#define TurnDetected xMessageRtrEncdBtn.TurnDetected

/*Imported Signals*/
extern HeatingSysQ RxedMessageToRtr;

#define SOLLtemperature RxedMessageToRtr.SOLLtemperature

#endif /* RTRENCDBTNQ_H_ */
